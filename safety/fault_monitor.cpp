#include "fault_monitor.h"
#include "arm.h"
#include "camera.h"

FaultCode FaultMonitor::Update(const Arm& arm, const Camera& camera) {
  if (CheckServoDisconnected(arm))     return FaultCode::ServoDisconnected;
  if (CheckExcessiveTorque(arm))       return FaultCode::ExcessiveTorque;
  if (CheckCameraDisconnected(camera)) return FaultCode::CameraDisconnected;

  return FaultCode::None;
}

bool FaultMonitor::CheckServoDisconnected(const Arm& arm) const {
  return !arm.IsConnected();
}

bool FaultMonitor::CheckExcessiveTorque(const Arm& arm) const {
  return arm.GetMaxTorque() > MaxTorque;
}

bool FaultMonitor::CheckCameraDisconnected(const Camera& camera) const {
  return !camera.IsConnected();
}
