// fault_monitor.h
#pragma once

#include "fault_code.h"

// Forward declarations — avoids fault_monitor.h depending on the full
// Arm/Camera class definitions, only their interfaces are needed here.
class Arm;
class Camera;

// Watches Arm/Camera state every control loop cycle and decides
// whether a fault is present. Runs identically in kNormal and
// kFaultTest mode — it has no knowledge of FaultInjector and no
// knowledge of whether a reading is "really" faulty or corrupted
// for a test. It only sees the same data the rest of the system sees.
class FaultMonitor {
 public:
  // Call once per control loop iteration. Returns the single
  // highest-priority fault currently detected, or FaultCode::kNone.
  FaultCode Update(const Arm& arm, const Camera& camera, double loop_dt);

 private:
  // --- actuator fault checks ---
  bool CheckServoDisconnected(const Arm& arm) const;
  bool CheckExcessiveTorque(const Arm& arm) const;

  // --- perception fault checks ---
  bool CheckCameraDisconnected(const Camera& camera) const;

  // more fault checks (and any state/thresholds they need) will be
  // added here as new FaultCodes come online

  // --- thresholds ---
  static constexpr double kMaxTorque = 1.0;  // Nm, tune to actual limit
};
