#include "arm.h"
#include "camera.h"
#include "fault_code.h"
#include "fault_injector.h"
#include "fault_monitor.h"
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <netinet/in.h>
#include <sstream>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>

#define ExcessiveTorqueTestCurrentAmps 3.0 // the safe threshold for motor current

// Flag to handle shutdowns
std::atomic<bool> g_shutdown_requested{false};
void SignalHandler(int signal) {
  g_shutdown_requested = true;
}

// All possible faults used for testing
FaultCode AllFaultCodes[] = {
    FaultCode::ServoDisconnected,
    FaultCode::ExcessiveTorque,
    FaultCode::CameraDisconnected,
};

// Helper function to create a FaultScenario, which is used by the FaultInjector for testing
FaultScenario ScenarioFor(FaultCode code) {
  FaultScenario scenario;
  scenario.code = code;
  if (code == FaultCode::ExcessiveTorque) {
    scenario.magnitude = ExcessiveTorqueTestCurrentAmps;
  }
  return scenario;
}

// Function called once on program startup to establish connection between main process and GUI process
// via TCP socket
int GuiStatusPort = 8765;
int GuiConnectTimeoutSec = 1;

int TryAcceptGuiClient(int server_fd) {
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(server_fd, &read_fds);
  timeval timeout{GuiConnectTimeoutSec, 0};

  int ready = select(server_fd + 1, &read_fds, nullptr, nullptr, &timeout);
  if (ready <= 0) return -1;
  return accept(server_fd, nullptr, nullptr);
}

// Primary function that handles socket creation and connecting to GUI process
int ConnectToGui() {
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  int reuse_addr = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(GuiStatusPort);
  bind(server_fd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr));
  listen(server_fd, 1);

  std::cout << "Checking for GUI on port " << GuiStatusPort << "..." << std::endl;
  int gui_client_fd = TryAcceptGuiClient(server_fd);
  std::cout << (gui_client_fd != -1 ? "GUI connected." : "No GUI detected — continuing without it.") << std::endl;
  close(server_fd);  // one-shot check; not accepting further clients

  return gui_client_fd;
}

// Instantaneously reads commands sent from the GUI (unless it has been terminated)
std::string ReadGuiCommand(int client_fd, bool* client_connected) {
  char buffer[64];
  ssize_t n = recv(client_fd, buffer, sizeof(buffer), MSG_DONTWAIT);
  if (n > 0) {
    return std::string(buffer, static_cast<size_t>(n));
  }
  if (n == 0 || (errno != EWOULDBLOCK && errno != EAGAIN)) {
    close(client_fd);
    *client_connected = false;
  }
  return "";
}

// Maps a fault name as sent by the GUI back to its FaultCode
FaultCode FaultCodeFromName(const std::string& name) {
  for (FaultCode code : AllFaultCodes) {
    if (name == FaultCodeToString(code)) return code;
  }
  return FaultCode::None;
}

int main() {
  int gui_client_fd = ConnectToGui();
  bool gui_connected = gui_client_fd != -1;

  FaultInjector fault_injector;
  FaultMonitor fault_monitor;
  Arm arm("/dev/ttyUSB0"); // U2D2 adapter serial port on Linux (could show as ttyUSB1 or other)
  arm.SetFaultInjector(&fault_injector);
  arm.Connect();

  Camera camera;
  camera.SetFaultInjector(&fault_injector);
  camera.Open(4);

  // Register signal handler
  std::signal(SIGINT, SignalHandler);   // ctrl+c
  std::signal(SIGTERM, SignalHandler);  // kill command

  double target_x;
  double target_y;

  // Arm homes itself on startup
  arm.Write("cartesian", 0.0, -50.0, 50, 2);
  arm.NonBlockingState(true);

  bool was_faulted = false;

  while (1) {
    if (g_shutdown_requested) {
      std::cout << "\nShutting down — moving arm to home position..." << std::endl;
      arm.Write("cartesian", 0.0, -50.0, 50, 2);
      if (gui_connected) close(gui_client_fd);
      break;
    }

    // Reviews status of hardware and detects if a fault is present
    arm.PollSafetyTelemetry();
    FaultCode fault = fault_monitor.Update(arm, camera);

    // Freezes while a fault is present/until cleared on the GUI
    bool freeze = fault != FaultCode::None;
    if (freeze) {
      if (!was_faulted) {
        std::cout << "!! FAULT DETECTED: " << FaultCodeToString(fault)
                  << " — holding position, no new commands will be issued." << std::endl;
        was_faulted = true;
      }
    } else {
      was_faulted = false;
    }
    
    // Handling GUI inputs
    if (gui_connected) {
      std::string command = ReadGuiCommand(gui_client_fd, &gui_connected);
      if (command == "ESTOP" || command == "QUIT") {
        std::cout << "\n" << command << " received from GUI — shutting down." << std::endl;
        g_shutdown_requested = true;
      } else if (command.rfind("INJECT:", 0) == 0) {
        FaultCode requested = FaultCodeFromName(command.substr(7));
        if (requested != FaultCode::None) {
          fault_injector.SetScenario(ScenarioFor(requested));
          std::cout << "GUI injected fault: " << FaultCodeToString(requested) << std::endl;
        }
      } else if (command == "CLEAR") {
        fault_injector.ClearScenario();
        std::cout << "GUI cleared the injected fault." << std::endl;
      }

      // Status updates sent every cycle: motor torques and connection statuses
      if (gui_connected) {
        auto [m1_connected, m2_connected, m3_connected] = arm.GetMotorConnections();
        auto [m1_torque, m2_torque, m3_torque] = arm.GetMotorTorques();
        std::ostringstream status;
        status << "STATUS:"
               << "m1=" << (m1_connected ? 1 : 0) << ",m2=" << (m2_connected ? 1 : 0) << ",m3=" << (m3_connected ? 1 : 0)
               << ",t1=" << m1_torque << ",t2=" << m2_torque << ",t3=" << m3_torque
               << ",cam=" << (camera.IsConnected() ? 1 : 0) << "\n";
        std::string status_str = status.str();
        send(gui_client_fd, status_str.c_str(), status_str.size(), 0);
      }
    }

    // Implementing the actual freeze (delays actuator commands)
    if (freeze) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Primary movement section: captures strawberry position and moves
    std::tuple<double, double, double> marker_position = camera.GetStrawberryPosition();
    target_x = std::get<2>(marker_position);
    target_y = std::get<1>(marker_position);

    std::cout << "Target Position in Camera Coords (cm): X=" << std::get<0>(marker_position) << ", Y=" << std::get<1>(marker_position) << ", Z=" << std::get<2>(marker_position) << std::endl;
    std::cout << "Target Arm Coords (cm): X=" << target_x << ", Y=" << target_y << std::endl;

    if (target_x == -1) {
      std::cout << "Failed to detect marker." << std::endl;
      continue;
    }

    target_x = target_x + CameraOffsetX - EndEffectorLength; // Accounting for physical hardware positions
    target_y = target_y + CameraOffsetY;
    arm.Write("cartesian", target_x, target_y, 50, 2);
  }

  return 0;
}
