#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "mavgs/frame.hpp"
#include "mavgs/link.hpp"
#include "mavgs/messages.hpp"
#include "mavgs/mission.hpp"
#include "mavgs/stats.hpp"
#include "mavgs/tlog.hpp"
#include "mavgs/vehicle_state.hpp"

namespace mavgs {

struct StationConfig {
  std::uint8_t sysid = 255;
  std::uint8_t compid = 190;                 ///< MAV_COMP_ID_MISSIONPLANNER
  std::optional<std::uint8_t> vehicle_sysid; ///< track this system; default: first autopilot seen
};

struct Event {
  std::uint64_t id = 0;
  std::uint64_t t_us = 0;
  std::string text;
};

enum class CommandStatus : std::uint8_t { Idle, Pending, Accepted, Rejected, TimedOut };

/// Ground-station core: owns protocol state and talks to a Link. Single
/// threaded; drive it by calling service() in a loop. Times are caller
/// supplied (monotonic microseconds), so tests can run it deterministically.
class Station {
 public:
  static constexpr std::uint64_t kHeartbeatPeriodUs = 1'000'000;
  static constexpr std::uint64_t kPingPeriodUs = 1'000'000;
  static constexpr std::uint64_t kLinkTimeoutUs = 3'000'000;
  static constexpr std::uint64_t kCommandRetryUs = 1'500'000;
  static constexpr int kCommandAttempts = 3;
  static constexpr std::size_t kMaxEvents = 64;

  Station(Link& link, StationConfig config, TlogWriter* recorder = nullptr);

  /// Receives for up to `wait` (then drains whatever else is queued), processes
  /// the frames and runs the periodic work: GCS heartbeat, ping, retries. Time
  /// spent blocked in `wait` is added to `now_us` for frames that arrive.
  void service(std::uint64_t now_us, std::chrono::milliseconds wait);

  // Operator actions. They return false (and log an event) if they cannot be
  // issued; acceptance by the vehicle is reported through command_status().
  [[nodiscard]] bool arm();
  [[nodiscard]] bool disarm();
  [[nodiscard]] bool takeoff(double altitude_m);
  [[nodiscard]] bool land();
  [[nodiscard]] bool return_to_launch();
  [[nodiscard]] bool start_mission();
  [[nodiscard]] bool upload_mission(std::vector<MissionItem> items);

  /// Adds an operator-visible message to the event log.
  void note(std::string text) { log(std::move(text)); }

  [[nodiscard]] const VehicleState& vehicle() const { return vehicle_; }
  [[nodiscard]] bool vehicle_online() const;
  [[nodiscard]] const LinkStats& stats() const { return stats_; }
  [[nodiscard]] const MissionUploader& uploader() const { return uploader_; }
  [[nodiscard]] std::size_t mission_size() const { return mission_size_; }
  [[nodiscard]] const std::deque<Event>& events() const { return events_; }
  [[nodiscard]] std::uint64_t event_count() const { return next_event_id_; }
  [[nodiscard]] CommandStatus command_status() const { return command_status_; }
  [[nodiscard]] const Link& link() const { return link_; }
  [[nodiscard]] std::uint64_t now_us() const { return now_us_; }

 private:
  void ingest(const Bytes& datagram);
  void handle(const Frame& frame, const Message& message);
  void periodic();
  bool send(const Message& message);
  bool send_command(CommandLong command, const char* label);
  void log(std::string text);
  void relay_uploader(const std::vector<Message>& messages);

  Link& link_;
  StationConfig config_;
  TlogWriter* recorder_;
  Parser parser_;
  Packer packer_;
  LinkStats stats_;
  VehicleState vehicle_;
  MissionUploader uploader_;
  std::size_t mission_size_ = 0;
  std::deque<Event> events_;
  std::uint64_t next_event_id_ = 0;
  std::uint64_t now_us_ = 0;

  std::optional<std::uint8_t> tracked_sysid_;
  bool was_online_ = false;
  std::uint64_t last_heartbeat_tx_us_ = 0;
  std::uint64_t last_ping_tx_us_ = 0;
  std::uint32_t ping_seq_ = 0;
  bool sent_first_ = false;
  MissionUploader::State last_upload_state_ = MissionUploader::State::Idle;

  CommandStatus command_status_ = CommandStatus::Idle;
  CommandLong pending_command_;
  std::string pending_label_;
  std::uint64_t command_tx_us_ = 0;
  int command_attempts_ = 0;
};

}  // namespace mavgs
