#pragma once

#include "fault_code.h"

class Arm;
class Camera;

// Watches Arm/Camera state every control loop cycle and decides whether a fault is present
class FaultMonitor {
 public:
  FaultCode Update(const Arm& arm, const Camera& camera);

 private:
  bool CheckServoDisconnected(const Arm& arm) const;
  bool CheckExcessiveTorque(const Arm& arm) const;
  bool CheckCameraDisconnected(const Camera& camera) const;

  static constexpr double MaxTorque = 1.0;
};
