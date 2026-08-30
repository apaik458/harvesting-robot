// fault_injector.h
#pragma once

#include <mutex>
#include "fault_code.h"

// Describes which fault to inject, and how severe (if relevant)
struct FaultScenario {
  FaultCode code = FaultCode::None;
  double magnitude = 0.0;
};

// Allows the user to simulate (inject) faults for testing
class FaultInjector {
 public:
  void SetScenario(const FaultScenario& scenario);
  void ClearScenario();
  double ApplyToValue(double real_value, FaultCode fault_type) const;
  bool IsBlocked(FaultCode fault_type) const;

 private:
  mutable std::mutex mutex_;
  FaultCode active_code_ = FaultCode::None;
  double active_magnitude_ = 0.0;
};
