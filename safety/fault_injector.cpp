// fault_injector.cpp
#include "fault_injector.h"

void FaultInjector::SetMode(SystemMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = mode;
}

SystemMode FaultInjector::GetMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

void FaultInjector::SetScenario(const FaultScenario& scenario) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_scenario_ = scenario;
}

void FaultInjector::ClearScenario() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_scenario_.reset();
}

FaultCode FaultInjector::ActiveFault() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_scenario_ ? active_scenario_->code : FaultCode::kNone;
}

double FaultInjector::ApplyToValue(double real_value, FaultCode fault_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == SystemMode::kNormal || !active_scenario_ || active_scenario_->code != fault_type) {
    return real_value;
  }

  switch (fault_type) {
    case FaultCode::kExcessiveTorque:
      // Report a fixed high load regardless of actual load.
      return active_scenario_->magnitude;

    default:
      return real_value;
  }
}

bool FaultInjector::IsBlocked(FaultCode fault_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == SystemMode::kNormal || !active_scenario_ || active_scenario_->code != fault_type) {
    return false;
  }

  switch (fault_type) {
    case FaultCode::kServoDisconnected:
    case FaultCode::kCameraDisconnected:
      return true;
    default:
      return false;
  }
}
