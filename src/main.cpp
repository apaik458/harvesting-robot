#include "arm.h"
#include "camera.h"
#include "fault_code.h"
#include "fault_injector.h"
#include "fault_monitor.h"
#include "system_mode.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <random>
#include <thread>
#include <vector>

// Set by the signal handler, consumed by the main loop. Signal handlers
// must stay minimal and avoid touching non-reentrant state — the old
// version called straight into Arm/the Dynamixel SDK from the handler,
// so a Ctrl+C landing mid-transaction (e.g. during the blocking homing
// write or a large-movement waypoint sequence) re-entered the same
// port_handler_/packet_handler_ state the main thread was using,
// corrupting the read and spinning forever on "Port is in use!". Setting
// only this flag here, and doing the actual homing from the main loop
// once it's between iterations, keeps all hardware I/O on one thread.
std::atomic<bool> g_shutdown_requested{false};

void SignalHandler(int signal) {
  g_shutdown_requested = true;
}

// Full set of faults the scripted HITL sequence cycles through. Extend
// this list as new FaultCodes come online.
constexpr FaultCode kAllFaultCodes[] = {
    FaultCode::kServoDisconnected,
    FaultCode::kExcessiveTorque,
    FaultCode::kCameraDisconnected,
};

constexpr double kNormalModeDurationSec = 30.0;
constexpr double kFaultFreezeDurationSec = 5.0;
constexpr double kFaultRecoverDurationSec = 10.0;
constexpr double kExcessiveTorqueTestCurrentAmps = 4.0;

// Phase of the scripted HITL fault-test sequence, once it has started
// (see kNormalModeDurationSec / kFaultFreezeDurationSec / kFaultRecoverDurationSec).
enum class TestPhase {
  kWarmup,      // still within the initial normal-operation window
  kFreezing,    // a fault is injected; the loop is holding position
  kRecovering,  // fault cleared; operating normally before the next fault
  kDone         // every fault has had its turn
};

// Scenario parameters to use for a given fault code in the scripted sequence.
FaultScenario ScenarioFor(FaultCode code) {
  FaultScenario scenario;
  scenario.code = code;
  if (code == FaultCode::kExcessiveTorque) {
    scenario.magnitude = kExcessiveTorqueTestCurrentAmps;
  }
  return scenario;
}

int main(int argc, char** argv) {
  bool fault_test_requested = false;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--fault-test") {
      fault_test_requested = true;
    }
  }

  // Always start clean — the scripted HITL sequence below (if requested)
  // switches to kFaultTest itself once the normal-operation warm-up ends.
  FaultInjector fault_injector(SystemMode::kNormal);
  FaultMonitor fault_monitor;

  Arm arm("/dev/ttyUSB0");
  arm.SetFaultInjector(&fault_injector);
  arm.Connect();

  // register signal handler
  std::signal(SIGINT, SignalHandler);   // ctrl+c
  std::signal(SIGTERM, SignalHandler);  // kill command

  Camera camera;
  camera.SetFaultInjector(&fault_injector);
  camera.Open(4);

  double target_x;
  double target_y;

  arm.Write("cartesian", 0.0, -50.0, 50, 2);
  arm.NonBlockingState(true);

  // --- Scripted HITL fault-test sequence state ---
  // Runs kNormalModeDurationSec of normal operation, then switches to
  // kFaultTest and cycles every known fault, one at a time in a
  // randomised order: inject -> freeze for kFaultFreezeDurationSec once
  // detected -> clear -> operate normally for kFaultRecoverDurationSec
  // -> inject the next one.
  std::vector<FaultCode> fault_order(std::begin(kAllFaultCodes), std::end(kAllFaultCodes));
  if (fault_test_requested) {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::shuffle(fault_order.begin(), fault_order.end(), rng);
    std::cout << "HITL fault-test sequence armed: " << kNormalModeDurationSec
              << "s normal operation, then " << fault_order.size() << " faults cycled in randomised order ("
              << kFaultFreezeDurationSec << "s frozen, " << kFaultRecoverDurationSec << "s recovery, each)."
              << std::endl;
  }
  TestPhase test_phase = TestPhase::kWarmup;
  size_t fault_index = 0;
  const auto test_start = std::chrono::steady_clock::now();
  auto phase_start = test_start;

  auto last_time = std::chrono::steady_clock::now();
  bool was_faulted = false;

  while (1) {
    if (g_shutdown_requested) {
      std::cout << "\nShutting down — moving arm to home position..." << std::endl;
      arm.NonBlockingState(true);
      arm.Write("cartesian", 0.0, -50.0, 50, 2);
      break;
    }

    auto now = std::chrono::steady_clock::now();
    double loop_dt = std::chrono::duration<double>(now - last_time).count();
    last_time = now;

    if (fault_test_requested && test_phase != TestPhase::kDone) {
      double elapsed_in_phase = std::chrono::duration<double>(now - phase_start).count();

      if (test_phase == TestPhase::kWarmup && elapsed_in_phase >= kNormalModeDurationSec) {
        fault_injector.SetMode(SystemMode::kFaultTest);
        std::cout << "\n=== Entering HITL fault-test sequence ===" << std::endl;
        if (!fault_order.empty()) {
          fault_injector.SetScenario(ScenarioFor(fault_order[0]));
          std::cout << "Injecting " << FaultCodeToString(fault_order[0])
                    << " — expect a " << kFaultFreezeDurationSec << "s freeze" << std::endl;
          test_phase = TestPhase::kFreezing;
        } else {
          test_phase = TestPhase::kDone;
        }
        phase_start = now;
      } else if (test_phase == TestPhase::kFreezing && elapsed_in_phase >= kFaultFreezeDurationSec) {
        fault_injector.ClearScenario();
        std::cout << "Fault cleared — resuming normal operation for " << kFaultRecoverDurationSec << "s" << std::endl;
        test_phase = TestPhase::kRecovering;
        phase_start = now;
      } else if (test_phase == TestPhase::kRecovering && elapsed_in_phase >= kFaultRecoverDurationSec) {
        fault_index++;
        if (fault_index < fault_order.size()) {
          fault_injector.SetScenario(ScenarioFor(fault_order[fault_index]));
          std::cout << "Injecting " << FaultCodeToString(fault_order[fault_index])
                    << " — expect a " << kFaultFreezeDurationSec << "s freeze" << std::endl;
          test_phase = TestPhase::kFreezing;
        } else {
          std::cout << "=== HITL fault-test sequence complete ===" << std::endl;
          test_phase = TestPhase::kDone;
        }
        phase_start = now;
      }
    }

    arm.PollSafetyTelemetry();
    FaultCode fault = fault_monitor.Update(arm, camera, loop_dt);

    // Freeze for real (unbounded) whenever a genuine fault is detected in
    // normal operation, or for the scripted kFaultFreezeDurationSec window
    // while the HITL sequence is deliberately holding one open.
    bool freeze = fault != FaultCode::kNone &&
                  (fault_injector.GetMode() == SystemMode::kNormal || test_phase == TestPhase::kFreezing);

    if (fault != FaultCode::kNone) {
      if (!was_faulted) {
        std::cout << "!! FAULT DETECTED: " << FaultCodeToString(fault)
                  << (freeze ? " — holding position, no new commands will be issued."
                             : " — continuing to operate normally.")
                  << std::endl;
        was_faulted = true;
      }
    } else {
      was_faulted = false;
    }

    if (freeze) {
      // Real safety response: hold position, issue no new commands.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    std::tuple<double, double, double> marker_position = camera.GetStrawberryPosition();
    target_x = std::get<2>(marker_position);
    target_y = std::get<1>(marker_position);

    std::cout << "Target Position in Camera Coords (cm): X=" << std::get<0>(marker_position) << ", Y=" << std::get<1>(marker_position) << ", Z=" << std::get<2>(marker_position) << std::endl;
    std::cout << "Target Arm Coords (cm): X=" << target_x << ", Y=" << target_y << std::endl;

    if (target_x == -1) {
      std::cout << "Failed to detect marker. Exiting." << std::endl;
      continue;
    }

    target_x = target_x + kCameraOffsetX - kEndEffectorLength;
    target_y = target_y + kCameraOffsetY;
    arm.Write("cartesian", target_x, target_y, 50, 2);
  }

  return 0;
}
