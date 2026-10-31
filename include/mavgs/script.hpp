#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mavgs/mission.hpp"
#include "mavgs/station.hpp"

namespace mavgs {

struct ScriptStep {
  enum class Kind : std::uint8_t { Arm, Disarm, Takeoff, Land, Rtl, StartMission, UploadMission, Wait, WaitDisarmed };
  Kind kind = Kind::Wait;
  double value = 0;                ///< altitude (Takeoff) or seconds (Wait)
  std::vector<MissionItem> items;  ///< UploadMission
  std::string label;
};

/// Parses a comma-separated step list:
/// @code
/// arm, disarm, takeoff:<alt_m>, land, rtl, start, mission:<file>, wait:<s>, wait-disarmed
/// @endcode
/// Mission files are read at parse time. Throws std::invalid_argument.
[[nodiscard]] std::vector<ScriptStep> parse_script(const std::string& text);

/// Runs steps one at a time against a Station, waiting for each command to be
/// acknowledged before moving on. Call advance() after every Station::service().
class ScriptRunner {
 public:
  explicit ScriptRunner(std::vector<ScriptStep> steps) : steps_(std::move(steps)) {}

  void advance(Station& station);

  [[nodiscard]] bool finished() const { return failed_ || index_ >= steps_.size(); }
  [[nodiscard]] bool failed() const { return failed_; }
  [[nodiscard]] const std::string& error() const { return error_; }

 private:
  void next();
  void fail(const std::string& reason);

  std::vector<ScriptStep> steps_;
  std::size_t index_ = 0;
  bool issued_ = false;
  bool saw_armed_ = false;
  std::uint64_t deadline_us_ = 0;
  bool failed_ = false;
  std::string error_;
};

}  // namespace mavgs
