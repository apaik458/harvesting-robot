// fault_code.h
#pragma once

// Represents the current fault state of the system.
// kNone == 0 so `if (fault_code)` reads as "a fault is active".
// Ordered roughly by severity/priority — FaultMonitor should report
// the highest-priority active fault if more than one condition is met.
enum class FaultCode {
  kNone = 0,

  // Actuator faults
  kServoDisconnected,   // comms timeout / no response from Dynamixel
  kExcessiveTorque,     // load/current reading exceeds safe threshold

  // Perception faults
  kCameraDisconnected,  // RealSense camera not opened / feed not available

  // more fault codes will be added here over time
};

// Human-readable name, useful for logging and the HITL test report.
inline const char* FaultCodeToString(FaultCode code) {
  switch (code) {
    case FaultCode::kNone:               return "NONE";
    case FaultCode::kServoDisconnected:  return "SERVO_DISCONNECTED";
    case FaultCode::kExcessiveTorque:    return "EXCESSIVE_TORQUE";
    case FaultCode::kCameraDisconnected: return "CAMERA_DISCONNECTED";
    default:                             return "UNKNOWN";
  }
}
