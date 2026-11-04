#include "mavgs/sim.hpp"

#include <algorithm>
#include <cmath>

#include "mavgs/enums.hpp"

namespace mavgs {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerDegree = 111319.49;
constexpr double kGravity = 9.80665;
constexpr double kMaxStep = 0.05;           // physics sub-step, s
constexpr double kMaxClimb = 3.0;           // m/s
constexpr double kMaxDescent = 1.5;         // m/s
constexpr double kLandSpeed = 0.8;          // m/s, below 3 m
constexpr double kRtlAltitude = 15.0;       // m above home
constexpr double kDefaultTakeoff = 10.0;    // m
constexpr double kArriveRadius = 1.5;       // m
constexpr double kBatteryFullMv = 16800.0;
constexpr double kBatteryEmptyMv = 13200.0;

double wrap_pi(double a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}

}  // namespace

VehicleSim::VehicleSim(SimConfig config)
    : config_(config), packer_(config.sysid, config.compid), rng_(config.seed == 0 ? 1 : config.seed) {}

double VehicleSim::next_random() {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return static_cast<double>(rng_) / 4294967296.0;
}

double VehicleSim::ground_speed() const { return std::hypot(vn_, ve_); }

std::int32_t VehicleSim::lat_e7() const {
  return static_cast<std::int32_t>(std::llround((config_.home_lat + north_ / kMetresPerDegree) * 1e7));
}

std::int32_t VehicleSim::lon_e7() const {
  const double scale = kMetresPerDegree * std::cos(config_.home_lat * kPi / 180.0);
  return static_cast<std::int32_t>(std::llround((config_.home_lon + east_ / scale) * 1e7));
}

void VehicleSim::local_from_wgs84(std::int32_t lat, std::int32_t lon, double& north,
                                  double& east) const {
  north = (lat / 1e7 - config_.home_lat) * kMetresPerDegree;
  east = (lon / 1e7 - config_.home_lon) * kMetresPerDegree * std::cos(config_.home_lat * kPi / 180.0);
}

void VehicleSim::say(const std::string& text) {
  log_.push_back(text);
  StatusText st;
  st.text = text.substr(0, 50);
  pending_text_.emplace_back(st);
}

std::vector<std::string> VehicleSim::take_log() {
  std::vector<std::string> out;
  out.swap(log_);
  return out;
}

void VehicleSim::set_mode(Mode mode) { mode_ = mode; }

Heartbeat VehicleSim::heartbeat() const {
  Heartbeat hb;
  hb.type = static_cast<std::uint8_t>(MavType::Quadrotor);
  hb.autopilot = static_cast<std::uint8_t>(MavAutopilot::Px4);
  hb.base_mode = base_mode::kCustomEnabled | (armed_ ? base_mode::kSafetyArmed : 0) |
                 (mode_ == Mode::Idle ? 0 : base_mode::kGuidedEnabled);
  hb.system_status = static_cast<std::uint8_t>(armed_ ? MavState::Active : MavState::Standby);
  std::uint8_t main = px4::kMainAuto;
  std::uint8_t sub = px4::kSubReady;
  switch (mode_) {
    case Mode::Idle:
      if (!armed_) {
        main = px4::kMainPosctl;
        sub = 0;
      }
      break;
    case Mode::Takeoff: sub = px4::kSubTakeoff; break;
    case Mode::Hold: sub = px4::kSubLoiter; break;
    case Mode::Mission: sub = px4::kSubMission; break;
    case Mode::Rtl: sub = px4::kSubRtl; break;
    case Mode::Land: sub = px4::kSubLand; break;
  }
  hb.custom_mode = px4::custom_mode(main, sub);
  return hb;
}

void VehicleSim::emit(const Message& message, std::vector<Bytes>& out) {
  Bytes wire = packer_.pack(message);  // sequence advances even if the frame is dropped
  if (config_.loss > 0.0 && next_random() < config_.loss) {
    ++dropped_;
    return;
  }
  out.push_back(std::move(wire));
}

// ---------------------------------------------------------------- physics

void VehicleSim::fly_towards(double target_n, double target_e, double target_up, double dt,
                             double max_down) {
  const double dn = target_n - north_;
  const double de = target_e - east_;
  const double dist = std::hypot(dn, de);
  double des_vn = 0, des_ve = 0;
  if (dist > 0.05) {
    const double speed = std::min(config_.cruise_speed, dist * 0.8);
    des_vn = dn / dist * speed;
    des_ve = de / dist * speed;
  }
  const double dz = target_up - up_;
  const double des_vu = std::clamp(dz * 0.8, -max_down, kMaxClimb);

  const double k = std::min(1.0, dt * 2.0);
  const double prev_vn = vn_;
  const double prev_ve = ve_;
  vn_ += (des_vn - vn_) * k;
  ve_ += (des_ve - ve_) * k;
  vu_ += (des_vu - vu_) * k;

  // Attitude from horizontal acceleration, yaw follows the direction of travel.
  const double an = (vn_ - prev_vn) / dt;
  const double ae = (ve_ - prev_ve) / dt;
  const double forward = an * std::cos(yaw_) + ae * std::sin(yaw_);
  const double right = -an * std::sin(yaw_) + ae * std::cos(yaw_);
  roll_ = std::atan2(right, kGravity);
  pitch_ = -std::atan2(forward, kGravity);
  if (ground_speed() > 1.0) {
    const double bearing = std::atan2(ve_, vn_);
    const double max_turn = (kPi / 2) * dt;
    yaw_ = wrap_pi(yaw_ + std::clamp(wrap_pi(bearing - yaw_), -max_turn, max_turn));
  }
}

double VehicleSim::descent_limit() const { return up_ > 3.0 ? kMaxDescent : kLandSpeed; }

void VehicleSim::begin_mission_item() {
  const MissionItem& item = mission_[mission_index_];
  hold_left_ = 0;
  switch (static_cast<MavCmd>(item.command)) {
    case MavCmd::NavTakeoff:
      target_n_ = north_;
      target_e_ = east_;
      target_up_ = item.z;
      break;
    case MavCmd::NavWaypoint:
      local_from_wgs84(item.x, item.y, target_n_, target_e_);
      target_up_ = item.z;
      hold_left_ = item.param1;
      break;
    case MavCmd::NavLand:
      if (item.x != 0 || item.y != 0) local_from_wgs84(item.x, item.y, target_n_, target_e_);
      target_up_ = 0;
      break;
    case MavCmd::NavReturnToLaunch:
      say("Return to launch");
      mission_active_ = false;
      set_mode(Mode::Rtl);
      target_up_ = std::max(up_, kRtlAltitude);
      break;
    default: break;
  }
}

void VehicleSim::advance_mission() {
  ++mission_index_;
  if (mission_index_ >= mission_.size()) {
    mission_active_ = false;
    mission_index_ = mission_.size() - 1;
    say("Mission finished, holding");
    set_mode(Mode::Hold);
    target_n_ = north_;
    target_e_ = east_;
    target_up_ = up_;
    return;
  }
  say("Reached waypoint " + std::to_string(mission_index_ - 1));
  begin_mission_item();
}

void VehicleSim::advance_physics(double dt) {
  const double prev_roll = roll_;
  const double prev_pitch = pitch_;
  const double prev_yaw = yaw_;

  if (!armed_ || mode_ == Mode::Idle) {
    vn_ = ve_ = vu_ = 0;
    roll_ = pitch_ = 0;
  } else {
    switch (mode_) {
      case Mode::Idle: break;
      case Mode::Takeoff:
      case Mode::Hold:
        fly_towards(target_n_, target_e_, target_up_, dt, kMaxDescent);
        if (mode_ == Mode::Takeoff && std::fabs(up_ - target_up_) < 0.5) {
          say("Takeoff complete, holding");
          set_mode(Mode::Hold);
        }
        break;
      case Mode::Mission: {
        const MissionItem& item = mission_[mission_index_];
        const auto cmd = static_cast<MavCmd>(item.command);
        if (cmd == MavCmd::NavLand) {
          const bool over = std::hypot(target_n_ - north_, target_e_ - east_) < kArriveRadius;
          fly_towards(target_n_, target_e_, over ? -1.0 : up_, dt, over ? descent_limit() : 0.0);
          if (over && up_ <= 0.05) {
            up_ = 0;
            armed_ = false;
            mission_active_ = false;
            set_mode(Mode::Idle);
            say("Landed, disarmed");
          }
        } else {
          fly_towards(target_n_, target_e_, target_up_, dt, kMaxDescent);
          const bool arrived = std::hypot(target_n_ - north_, target_e_ - east_) < kArriveRadius &&
                               std::fabs(target_up_ - up_) < 1.0;
          if (arrived) {
            hold_left_ -= dt;
            if (hold_left_ <= 0) advance_mission();
          }
        }
        break;
      }
      case Mode::Rtl: {
        const double home_dist = std::hypot(north_, east_);
        if (home_dist > kArriveRadius) {
          fly_towards(0, 0, std::max(target_up_, kRtlAltitude), dt, kMaxDescent);
        } else {
          fly_towards(0, 0, -1.0, dt, descent_limit());
          if (up_ <= 0.05) {
            up_ = 0;
            armed_ = false;
            set_mode(Mode::Idle);
            say("Landed, disarmed");
          }
        }
        break;
      }
      case Mode::Land:
        fly_towards(north_, east_, -1.0, dt, descent_limit());
        if (up_ <= 0.05) {
          up_ = 0;
          armed_ = false;
          set_mode(Mode::Idle);
          say("Landed, disarmed");
        }
        break;
    }
    north_ += vn_ * dt;
    east_ += ve_ * dt;
    up_ = std::max(0.0, up_ + vu_ * dt);
    if (up_ == 0.0 && vu_ < 0) vu_ = 0;
  }
  yaw_ = wrap_pi(yaw_);
  roll_rate_ = (roll_ - prev_roll) / dt;
  pitch_rate_ = (pitch_ - prev_pitch) / dt;
  yaw_rate_ = wrap_pi(yaw_ - prev_yaw) / dt;
  charge_ = std::max(0.0, charge_ - dt * (armed_ ? (airborne() ? 0.06 : 0.01) : 0.0));
}

// ---------------------------------------------------------------- telemetry

std::vector<Bytes> VehicleSim::step(std::uint64_t now_us) {
  std::vector<Bytes> out;
  if (!started_) {
    started_ = true;
    start_us_ = last_us_ = now_us;
    for (Stream* s : {&heartbeat_, &sys_status_, &attitude_, &position_, &gps_, &hud_, &mission_current_}) {
      s->next_us = now_us;
    }
  }

  double remaining = std::min(static_cast<double>(now_us - last_us_) / 1e6, 1.0) * config_.speed;
  last_us_ = now_us;
  while (remaining > 1e-9) {
    const double dt = std::min(remaining, kMaxStep);
    advance_physics(dt);
    remaining -= dt;
  }

  const auto due = [&](Stream& s) {
    if (now_us < s.next_us) return false;
    s.next_us += s.period_us;
    if (s.next_us <= now_us) s.next_us = now_us + s.period_us;
    return true;
  };
  const auto boot_ms = static_cast<std::uint32_t>((now_us - start_us_) / 1000);

  for (const Message& m : pending_text_) emit(m, out);
  pending_text_.clear();

  if (due(heartbeat_)) emit(heartbeat(), out);
  if (due(sys_status_)) {
    SysStatus s;
    s.sensors_present = s.sensors_enabled = s.sensors_health = 0x2F;
    s.load = 180;
    const double mv = kBatteryEmptyMv + (kBatteryFullMv - kBatteryEmptyMv) * charge_ / 100.0;
    s.voltage_battery = static_cast<std::uint16_t>(mv);
    s.current_battery = static_cast<std::int16_t>(armed_ ? (airborne() ? 1050 : 300) : 40);
    s.battery_remaining = static_cast<std::int8_t>(std::lround(charge_));
    emit(s, out);
  }
  if (due(attitude_)) {
    Attitude a;
    a.time_boot_ms = boot_ms;
    a.roll = static_cast<float>(roll_);
    a.pitch = static_cast<float>(pitch_);
    a.yaw = static_cast<float>(yaw_);
    a.rollspeed = static_cast<float>(roll_rate_);
    a.pitchspeed = static_cast<float>(pitch_rate_);
    a.yawspeed = static_cast<float>(yaw_rate_);
    emit(a, out);
  }
  if (due(position_)) {
    GlobalPositionInt p;
    p.time_boot_ms = boot_ms;
    p.lat = lat_e7();
    p.lon = lon_e7();
    p.alt = static_cast<std::int32_t>(std::llround((config_.home_alt_msl + up_) * 1000.0));
    p.relative_alt = static_cast<std::int32_t>(std::llround(up_ * 1000.0));
    p.vx = static_cast<std::int16_t>(std::lround(vn_ * 100.0));
    p.vy = static_cast<std::int16_t>(std::lround(ve_ * 100.0));
    p.vz = static_cast<std::int16_t>(std::lround(-vu_ * 100.0));
    double hdg = yaw_ * 180.0 / kPi;
    if (hdg < 0) hdg += 360.0;
    p.hdg = static_cast<std::uint16_t>(std::lround(hdg * 100.0) % 36000);
    emit(p, out);
  }
  if (due(gps_)) {
    GpsRawInt g;
    g.time_usec = now_us;
    g.lat = lat_e7();
    g.lon = lon_e7();
    g.alt = static_cast<std::int32_t>(std::llround((config_.home_alt_msl + up_) * 1000.0));
    g.eph = 80;
    g.epv = 120;
    g.vel = static_cast<std::uint16_t>(std::lround(ground_speed() * 100.0));
    g.fix_type = 3;
    g.satellites_visible = 12;
    emit(g, out);
  }
  if (due(hud_)) {
    VfrHud h;
    h.airspeed = h.groundspeed = static_cast<float>(ground_speed());
    h.alt = static_cast<float>(config_.home_alt_msl + up_);
    h.climb = static_cast<float>(vu_);
    double hdg = yaw_ * 180.0 / kPi;
    if (hdg < 0) hdg += 360.0;
    h.heading = static_cast<std::int16_t>(std::lround(hdg) % 360);
    h.throttle = static_cast<std::uint16_t>(
        armed_ ? std::clamp(std::lround(45 + vu_ * 8.0), 0L, 100L) : 0L);
    emit(h, out);
  }
  if (due(mission_current_) && !mission_.empty()) {
    emit(MissionCurrent{static_cast<std::uint16_t>(mission_index_)}, out);
  }
  return out;
}

// ---------------------------------------------------------------- protocol

void VehicleSim::on_command(const CommandLong& cmd, std::vector<Message>& replies) {
  MavResult result = MavResult::Accepted;
  switch (static_cast<MavCmd>(cmd.command)) {
    case MavCmd::ComponentArmDisarm:
      if (cmd.param1 > 0.5F) {
        if (!armed_) {
          armed_ = true;
          set_mode(Mode::Idle);
          say("Armed");
        }
      } else if (armed_) {
        if (airborne()) {
          result = MavResult::Denied;
        } else {
          armed_ = false;
          set_mode(Mode::Idle);
          mission_active_ = false;
          say("Disarmed");
        }
      }
      break;
    case MavCmd::NavTakeoff:
      if (!armed_ || airborne()) {
        result = MavResult::Denied;
      } else {
        const double amsl = std::isnan(cmd.param7) ? config_.home_alt_msl + kDefaultTakeoff : cmd.param7;
        target_up_ = std::clamp(amsl - config_.home_alt_msl, 2.0, 500.0);
        target_n_ = north_;
        target_e_ = east_;
        mission_active_ = false;
        set_mode(Mode::Takeoff);
        say("Takeoff to " + std::to_string(static_cast<int>(std::lround(target_up_))) + " m");
      }
      break;
    case MavCmd::NavLand:
      if (!armed_) {
        result = MavResult::Denied;
      } else {
        mission_active_ = false;
        set_mode(Mode::Land);
        say("Landing");
      }
      break;
    case MavCmd::NavReturnToLaunch:
      if (!armed_) {
        result = MavResult::Denied;
      } else {
        mission_active_ = false;
        target_up_ = std::max(up_, kRtlAltitude);
        set_mode(Mode::Rtl);
        say("Return to launch");
      }
      break;
    case MavCmd::MissionStart:
      if (!armed_ || mission_.empty()) {
        result = MavResult::Denied;
      } else {
        mission_index_ = 0;
        mission_active_ = true;
        say("Mission started");
        set_mode(Mode::Mission);
        begin_mission_item();
      }
      break;
    default: result = MavResult::Unsupported; break;
  }
  replies.emplace_back(CommandAck{cmd.command, static_cast<std::uint8_t>(result)});
}

void VehicleSim::on_message(const Frame& frame, const Message& message, std::vector<Message>& replies) {
  if (const auto* cmd = std::get_if<CommandLong>(&message)) {
    if (cmd->target_system == config_.sysid || cmd->target_system == 0) on_command(*cmd, replies);
  } else if (const auto* ping = std::get_if<Ping>(&message)) {
    if (ping->target_system == 0 && ping->target_component == 0) {
      replies.emplace_back(Ping{ping->time_usec, ping->seq, frame.sysid, frame.compid});
    }
  } else if (const auto* count = std::get_if<MissionCount>(&message)) {
    if (count->target_system != config_.sysid) return;
    if (count->count == 0 || count->count > kMaxMissionItems || (armed_ && mission_active_)) {
      replies.emplace_back(MissionAck{frame.sysid, frame.compid, static_cast<std::uint8_t>(MissionResult::Error)});
      receiving_ = false;
      return;
    }
    receiving_ = true;
    expected_count_ = count->count;
    incoming_.clear();
    replies.emplace_back(MissionRequestInt{0, frame.sysid, frame.compid});
  } else if (const auto* item = std::get_if<MissionItemInt>(&message)) {
    if (!receiving_ || item->target_system != config_.sysid) return;
    if (item->seq != incoming_.size()) {
      replies.emplace_back(MissionRequestInt{static_cast<std::uint16_t>(incoming_.size()), frame.sysid, frame.compid});
      return;
    }
    MissionItem stored;
    stored.command = item->command;
    stored.frame = item->frame;
    stored.param1 = item->param1;
    stored.param2 = item->param2;
    stored.param3 = item->param3;
    stored.param4 = item->param4;
    stored.x = item->x;
    stored.y = item->y;
    stored.z = item->z;
    incoming_.push_back(stored);
    if (incoming_.size() == expected_count_) {
      mission_ = std::move(incoming_);
      incoming_.clear();
      mission_index_ = 0;
      receiving_ = false;
      say("Mission received: " + std::to_string(mission_.size()) + " items");
      replies.emplace_back(MissionAck{frame.sysid, frame.compid, static_cast<std::uint8_t>(MissionResult::Accepted)});
    } else {
      replies.emplace_back(MissionRequestInt{static_cast<std::uint16_t>(incoming_.size()), frame.sysid, frame.compid});
    }
  }
}

std::vector<Bytes> VehicleSim::handle(const Bytes& datagram) {
  std::vector<Message> replies;
  for (const auto& parsed : parser_.feed(datagram)) {
    if (const auto message = decode(parsed.frame)) on_message(parsed.frame, *message, replies);
  }
  std::vector<Bytes> out;
  for (const Message& m : replies) emit(m, out);
  return out;
}

}  // namespace mavgs
