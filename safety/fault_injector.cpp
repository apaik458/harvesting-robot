#include "fault_injector.h"

// Sets an active fault
void FaultInjector::SetScenario(const FaultScenario& scenario) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_code_ = scenario.code;
  active_magnitude_ = scenario.magnitude;
}

// Clears an active fault
void FaultInjector::ClearScenario() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_code_ = FaultCode::None;
  active_magnitude_ = 0.0;
}

// Used to overwrite a hardware value with a simulated value, e.g. to simulate a large motor current
double FaultInjector::ApplyToValue(double real_value, FaultCode fault_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_code_ != fault_type) {
    return real_value;
  }

  switch (fault_type) {
    case FaultCode::ExcessiveTorque:
      // Report a fixed high load regardless of actual load
      return active_magnitude_;

    default:
      return real_value;
  }
}

// Used to simulate fault boolean conditions, e.g. to simulate servo disconnection
bool FaultInjector::IsBlocked(FaultCode fault_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_code_ != fault_type) {
    return false;
  }

  switch (fault_type) {
    case FaultCode::ServoDisconnected:
    case FaultCode::CameraDisconnected:
      return true;
    default:
      return false;
  }
}
