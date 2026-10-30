#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mavgs/frame.hpp"
#include "mavgs/messages.hpp"
#include "mavgs/mission.hpp"

namespace mavgs {

struct SimConfig {
  std::uint8_t sysid = 1;
  std::uint8_t compid = 1;
  double home_lat = 47.397742;  ///< PX4 SITL default home (Zurich)
  double home_lon = 8.545594;
  double home_alt_msl = 488.0;
  double speed = 1.0;           ///< simulation time multiplier
  double loss = 0.0;            ///< probability of dropping each outgoing frame
  std::uint32_t seed = 1;
  double cruise_speed = 8.0;    ///< m/s horizontal
};

/// A small kinematic multicopter that speaks MAVLink like a PX4 vehicle: it
/// streams telemetry, accepts arm/disarm/takeoff/land/RTL commands and the
/// mission upload protocol, and flies uploaded missions. Pure logic with
/// caller-supplied time; the executable wires it to a UDP socket.
class VehicleSim {
 public:
  enum class Mode { Idle, Takeoff, Hold, Mission, Rtl, Land };

  explicit VehicleSim(SimConfig config);

  /// Advances to `now_us` and returns the wire frames due for transmission.
  [[nodiscard]] std::vector<Bytes> step(std::uint64_t now_us);
  /// Processes datagrams from the ground station and returns the replies.
  [[nodiscard]] std::vector<Bytes> handle(const Bytes& datagram);
  /// Status lines produced since the last call (also streamed as STATUSTEXT).
  [[nodiscard]] std::vector<std::string> take_log();

  [[nodiscard]] bool armed() const { return armed_; }
  [[nodiscard]] Mode mode() const { return mode_; }
  [[nodiscard]] double altitude() const { return up_; }
  [[nodiscard]] double north() const { return north_; }
  [[nodiscard]] double east() const { return east_; }
  [[nodiscard]] double ground_speed() const;
  [[nodiscard]] std::size_t mission_size() const { return mission_.size(); }
  [[nodiscard]] std::size_t mission_index() const { return mission_index_; }
  [[nodiscard]] std::uint64_t frames_dropped() const { return dropped_; }

 private:
  struct Stream {
    std::uint64_t period_us;
    std::uint64_t next_us;
  };

  void advance_physics(double dt);
  void fly_towards(double target_n, double target_e, double target_up, double dt, double max_down);
  [[nodiscard]] double descent_limit() const;
  void begin_mission_item();
  void advance_mission();
  void set_mode(Mode mode);
  void say(const std::string& text);
  void emit(const Message& message, std::vector<Bytes>& out);
  void on_command(const CommandLong& cmd, std::vector<Message>& replies);
  void on_message(const Frame& frame, const Message& message, std::vector<Message>& replies);
  [[nodiscard]] double next_random();
  [[nodiscard]] std::int32_t lat_e7() const;
  [[nodiscard]] std::int32_t lon_e7() const;
  [[nodiscard]] bool airborne() const { return up_ > 0.3; }
  [[nodiscard]] Heartbeat heartbeat() const;
  void local_from_wgs84(std::int32_t lat, std::int32_t lon, double& north, double& east) const;

  SimConfig config_;
  Packer packer_;
  Parser parser_;
  std::vector<std::string> log_;
  std::vector<Message> pending_text_;
  std::uint32_t rng_;
  std::uint64_t dropped_ = 0;

  bool started_ = false;
  std::uint64_t last_us_ = 0;
  std::uint64_t start_us_ = 0;
  Stream heartbeat_{1'000'000, 0};
  Stream sys_status_{1'000'000, 0};
  Stream attitude_{100'000, 0};
  Stream position_{200'000, 0};
  Stream gps_{500'000, 0};
  Stream hud_{200'000, 0};
  Stream mission_current_{1'000'000, 0};

  // Kinematic state in a local frame around home (north/east/up, metres).
  bool armed_ = false;
  Mode mode_ = Mode::Idle;
  double north_ = 0, east_ = 0, up_ = 0;
  double vn_ = 0, ve_ = 0, vu_ = 0;
  double roll_ = 0, pitch_ = 0, yaw_ = 0;
  double roll_rate_ = 0, pitch_rate_ = 0, yaw_rate_ = 0;
  double target_n_ = 0, target_e_ = 0, target_up_ = 0;
  double charge_ = 100.0;  ///< battery %
  double hold_left_ = 0;

  std::vector<MissionItem> mission_;
  std::size_t mission_index_ = 0;
  bool mission_active_ = false;

  // Upload session.
  bool receiving_ = false;
  std::size_t expected_count_ = 0;
  std::vector<MissionItem> incoming_;
};

}  // namespace mavgs
