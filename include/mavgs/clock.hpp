#pragma once

#include <chrono>
#include <cstdint>

namespace mavgs {

/// Monotonic microseconds since an arbitrary epoch.
[[nodiscard]] inline std::uint64_t monotonic_us() {
  using namespace std::chrono;
  return static_cast<std::uint64_t>(
      duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

/// Wall-clock microseconds since the Unix epoch.
[[nodiscard]] inline std::uint64_t wall_us() {
  using namespace std::chrono;
  return static_cast<std::uint64_t>(
      duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
}

}  // namespace mavgs
