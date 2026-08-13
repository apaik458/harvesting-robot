// fault_injector.h
#pragma once

#include <mutex>
#include <optional>
#include "fault_code.h"
#include "system_mode.h"

// Describes a single fault to inject: which fault, and how severe.
// magnitude is interpreted differently depending on the fault code
// (e.g. a fixed value for kExcessiveTorque). Codes that don't need
// a magnitude ignore it.
struct FaultScenario {
  FaultCode code = FaultCode::kNone;
  double magnitude = 0.0;
};

// Sits between Arm/Camera and the real hardware reads. In kNormal mode
// it is a no-op pass-through. In kFaultTest mode, it corrupts values
// or reports connection loss for whichever FaultCode is currently
// active. Arm and Camera call this at their raw hardware read points
// and are otherwise unaware it exists.
class FaultInjector {
 public:
  explicit FaultInjector(SystemMode mode) : mode_(mode) {}

  // Mode and scenario can be changed live by a scripted HITL test sequence
  // running on the main thread while Arm/Camera read them from their own
  // threads (e.g. Camera's capture thread), so both are mutex-guarded.
  void SetMode(SystemMode mode);
  SystemMode GetMode() const;

  void SetScenario(const FaultScenario& scenario);
  void ClearScenario();
  FaultCode ActiveFault() const;

  // For faults that corrupt a numeric reading (position, torque, depth).
  // Returns real_value unchanged unless fault_type matches the active
  // scenario, in which case it applies that scenario's corruption.
  double ApplyToValue(double real_value, FaultCode fault_type) const;

  // For faults that represent total loss of a signal rather than a
  // corrupted value (disconnection, frame freeze). Returns true if
  // the caller should treat this read as failed/unavailable.
  bool IsBlocked(FaultCode fault_type) const;

 private:
  mutable std::mutex mutex_;
  SystemMode mode_;
  std::optional<FaultScenario> active_scenario_;
};
