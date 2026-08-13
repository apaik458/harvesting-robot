// fault_monitor.cpp
#include "fault_monitor.h"
#include "arm.h"
#include "camera.h"

FaultCode FaultMonitor::Update(const Arm& arm, const Camera& camera, double loop_dt) {
  // Priority order: actuator faults outrank perception faults —
  // a runaway/disconnected servo is more urgent than a lost detection.
  if (CheckServoDisconnected(arm))     return FaultCode::kServoDisconnected;
  if (CheckExcessiveTorque(arm))       return FaultCode::kExcessiveTorque;
  if (CheckCameraDisconnected(camera)) return FaultCode::kCameraDisconnected;

  return FaultCode::kNone;
}

bool FaultMonitor::CheckServoDisconnected(const Arm& arm) const {
  return !arm.IsConnected();  // assumes Arm exposes a connection status
}

bool FaultMonitor::CheckExcessiveTorque(const Arm& arm) const {
  return arm.GetTorque() > kMaxTorque;
}

bool FaultMonitor::CheckCameraDisconnected(const Camera& camera) const {
  return !camera.IsConnected();
}
