#pragma once

#include <cstdint>
#include <string>

#include "mavgs/station.hpp"

namespace mavgs {

struct RenderOptions {
  bool color = false;
  std::size_t max_events = 6;
  std::uint64_t origin_us = 0;  ///< event timestamps are shown relative to this
};

/// Full-screen telemetry dashboard as plain text (ANSI colors if requested).
/// Lines are separated by '\n'; no cursor control is emitted.
[[nodiscard]] std::string render_dashboard(const Station& station, const RenderOptions& options);

/// One-line status summary for log-style output.
[[nodiscard]] std::string render_status_line(const Station& station, double elapsed_s);

/// "[  12.3s] text" for an event relative to `origin_us`.
[[nodiscard]] std::string render_event(const Event& event, std::uint64_t origin_us);

}  // namespace mavgs
