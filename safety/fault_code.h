#pragma once

// Represents the current fault state of the system
enum class FaultCode {
  None = 0,
  ServoDisconnected,
  ExcessiveTorque,
  CameraDisconnected,

  // More faults may be added to this system
};

// Used for logging
inline const char* FaultCodeToString(FaultCode code) {
  switch (code) {
    case FaultCode::None:               return "NONE";
    case FaultCode::ServoDisconnected:  return "SERVO_DISCONNECTED";
    case FaultCode::ExcessiveTorque:    return "EXCESSIVE_TORQUE";
    case FaultCode::CameraDisconnected: return "CAMERA_DISCONNECTED";
    default:                             return "UNKNOWN";
  }
}
