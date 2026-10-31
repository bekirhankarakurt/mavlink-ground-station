#pragma once

#include <cstdint>
#include <iosfwd>
#include <stdexcept>
#include <string>
#include <vector>

#include "mavgs/messages.hpp"

namespace mavgs {

/// A mission item in the form it is uploaded (MISSION_ITEM_INT without addressing).
struct MissionItem {
  std::uint16_t command = 0;
  std::uint8_t frame = 0;
  float param1 = 0, param2 = 0, param3 = 0, param4 = 0;
  std::int32_t x = 0, y = 0;  ///< degE7
  float z = 0;                ///< m, relative to home
};

class MissionParseError : public std::runtime_error {
 public:
  MissionParseError(int line, const std::string& what)
      : std::runtime_error("line " + std::to_string(line) + ": " + what), line_(line) {}
  [[nodiscard]] int line() const { return line_; }

 private:
  int line_;
};

[[nodiscard]] inline bool operator==(const MissionItem& a, const MissionItem& b) {
  return a.command == b.command && a.frame == b.frame && a.param1 == b.param1 &&
         a.param2 == b.param2 && a.param3 == b.param3 && a.param4 == b.param4 && a.x == b.x &&
         a.y == b.y && a.z == b.z;
}

inline constexpr std::size_t kMaxMissionItems = 700;

/// Parses the plain-text mission format:
///
/// @code
/// takeoff <alt_m>
/// waypoint <lat> <lon> <alt_m> [hold_s]
/// land [<lat> <lon>]
/// rtl
/// @endcode
///
/// '#' starts a comment. Throws MissionParseError with the offending line.
[[nodiscard]] std::vector<MissionItem> parse_mission(std::istream& in);
[[nodiscard]] std::vector<MissionItem> load_mission(const std::string& path);

/// Client side of the MAVLink mission upload protocol
/// (COUNT -> REQUEST_INT -> ITEM_INT ... -> ACK) with retry on timeout.
/// Pure state machine: it returns messages to send and never touches I/O.
class MissionUploader {
 public:
  enum class State : std::uint8_t { Idle, Uploading, Done, Failed };

  static constexpr std::uint64_t kRetryUs = 1'000'000;
  static constexpr int kMaxRetries = 5;

  MissionUploader(std::uint8_t target_system, std::uint8_t target_component)
      : target_system_(target_system), target_component_(target_component) {}

  void set_target(std::uint8_t system, std::uint8_t component) {
    target_system_ = system;
    target_component_ = component;
  }

  [[nodiscard]] std::vector<Message> start(std::vector<MissionItem> items, std::uint64_t now_us);
  [[nodiscard]] std::vector<Message> on_message(const Message& message, std::uint64_t now_us);
  [[nodiscard]] std::vector<Message> poll(std::uint64_t now_us);

  [[nodiscard]] State state() const { return state_; }
  [[nodiscard]] const std::string& error() const { return error_; }
  [[nodiscard]] std::size_t total() const { return items_.size(); }
  [[nodiscard]] std::size_t sent() const { return sent_; }

 private:
  void fail(std::string reason);

  std::uint8_t target_system_;
  std::uint8_t target_component_;
  State state_ = State::Idle;
  std::string error_;
  std::vector<MissionItem> items_;
  std::size_t sent_ = 0;
  std::uint64_t last_tx_us_ = 0;
  int retries_ = 0;
  Message last_tx_;
};

[[nodiscard]] MissionItemInt to_wire(const MissionItem& item, std::uint16_t seq,
                                     std::uint8_t target_system, std::uint8_t target_component);

}  // namespace mavgs
