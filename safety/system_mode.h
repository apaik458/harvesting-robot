// system_mode.h
#pragma once

// Whether the system is running normally, or in a mode where
// FaultInjector is permitted to corrupt/block hardware reads.
enum class SystemMode {
  kNormal,
  kFaultTest
};
