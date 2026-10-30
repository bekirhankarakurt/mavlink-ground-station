#include "mavgs/station.hpp"

#include <cmath>
#include <limits>

#include "mavgs/clock.hpp"
#include "mavgs/enums.hpp"

namespace mavgs {

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr int kMaxDatagramsPerService = 256;

const char* result_name(std::uint8_t result) {
  switch (static_cast<MavResult>(result)) {
    case MavResult::Accepted: return "ACCEPTED";
    case MavResult::TemporarilyRejected: return "TEMPORARILY_REJECTED";
    case MavResult::Denied: return "DENIED";
    case MavResult::Unsupported: return "UNSUPPORTED";
    case MavResult::Failed: return "FAILED";
    case MavResult::InProgress: return "IN_PROGRESS";
    case MavResult::Cancelled: return "CANCELLED";
  }
  return "UNKNOWN";
}

const char* command_name(std::uint16_t command) {
  switch (static_cast<MavCmd>(command)) {
    case MavCmd::NavReturnToLaunch: return "RTL";
    case MavCmd::NavLand: return "LAND";
    case MavCmd::NavTakeoff: return "TAKEOFF";
    case MavCmd::MissionStart: return "MISSION_START";
    case MavCmd::ComponentArmDisarm: return "ARM/DISARM";
    default: return "COMMAND";
  }
}

}  // namespace

Station::Station(Link& link, StationConfig config, TlogWriter* recorder)
    : link_(link),
      config_(config),
      recorder_(recorder),
      packer_(config.sysid, config.compid),
      uploader_(config.vehicle_sysid.value_or(1), 1),
      tracked_sysid_(config.vehicle_sysid) {}

void Station::log(std::string text) {
  events_.push_back({next_event_id_++, now_us_, std::move(text)});
  if (events_.size() > kMaxEvents) events_.pop_front();
}

bool Station::send(const Message& message) { return link_.send(packer_.pack(message)); }

bool Station::vehicle_online() const {
  return vehicle_.seen && now_us_ - vehicle_.last_heartbeat_us < kLinkTimeoutUs;
}

void Station::service(std::uint64_t now_us, std::chrono::milliseconds wait) {
  now_us_ = now_us;
  const auto entered = std::chrono::steady_clock::now();
  // Time spent blocked in `wait` counts, so timestamps and latency samples stay accurate.
  const auto advance_clock = [&] {
    if (wait.count() > 0) {
      const auto blocked = std::chrono::steady_clock::now() - entered;
      now_us_ = now_us + static_cast<std::uint64_t>(
                             std::chrono::duration_cast<std::chrono::microseconds>(blocked).count());
    }
  };

  Bytes datagram;
  auto timeout = wait;
  for (int i = 0; i < kMaxDatagramsPerService && link_.receive(datagram, timeout); ++i) {
    timeout = std::chrono::milliseconds(0);
    advance_clock();
    ingest(datagram);
  }
  advance_clock();
  periodic();
}

void Station::ingest(const Bytes& datagram) {
  for (const auto& parsed : parser_.feed(datagram)) {
    const Frame& frame = parsed.frame;
    if (frame.sysid == config_.sysid) continue;  // echo of another GCS with our id

    stats_.on_frame(frame.sysid, frame.compid, frame.seq, parsed.raw.size());
    if (recorder_ != nullptr) recorder_->append(wall_us(), parsed.raw);

    const auto message = decode(frame);
    if (message) handle(frame, *message);
  }
  stats_.on_crc_errors(parser_.stats().crc_errors);
}

void Station::handle(const Frame& frame, const Message& message) {
  if (const auto* hb = std::get_if<Heartbeat>(&message)) {
    const bool is_autopilot = hb->type != static_cast<std::uint8_t>(MavType::Gcs) &&
                              hb->autopilot != static_cast<std::uint8_t>(MavAutopilot::Invalid);
    if (!tracked_sysid_ && is_autopilot) {
      tracked_sysid_ = frame.sysid;
      log("vehicle detected: system " + std::to_string(frame.sysid));
    }
    if (tracked_sysid_ && frame.sysid == *tracked_sysid_ && is_autopilot) {
      const bool first = !vehicle_.seen;
      vehicle_.sysid = frame.sysid;
      vehicle_.compid = frame.compid;
      uploader_.set_target(frame.sysid, frame.compid);
      if (first) was_online_ = true;
      const bool was_armed = vehicle_.armed();
      vehicle_.apply(message, now_us_);
      if (!first && was_armed != vehicle_.armed()) log(vehicle_.armed() ? "vehicle armed" : "vehicle disarmed");
    }
    return;
  }

  if (!tracked_sysid_ || frame.sysid != *tracked_sysid_) return;

  if (const auto* ack = std::get_if<CommandAck>(&message)) {
    if (command_status_ == CommandStatus::Pending && ack->command == pending_command_.command) {
      const bool ok = ack->result == static_cast<std::uint8_t>(MavResult::Accepted) ||
                      ack->result == static_cast<std::uint8_t>(MavResult::InProgress);
      command_status_ = ok ? CommandStatus::Accepted : CommandStatus::Rejected;
    }
    log(std::string(command_name(ack->command)) + ": " + result_name(ack->result));
  } else if (const auto* text = std::get_if<StatusText>(&message)) {
    log("vehicle: " + text->text);
  } else if (const auto* ping = std::get_if<Ping>(&message)) {
    if (ping->target_system == config_.sysid && ping->seq < ping_seq_ && !link_.is_replay() &&
        now_us_ >= ping->time_usec) {
      stats_.on_latency_sample(now_us_ - ping->time_usec);
    }
  } else if (std::holds_alternative<MissionRequestInt>(message) ||
             std::holds_alternative<MissionAck>(message)) {
    relay_uploader(uploader_.on_message(message, now_us_));
  } else {
    vehicle_.apply(message, now_us_);
  }
}

void Station::relay_uploader(const std::vector<Message>& messages) {
  for (const auto& m : messages) (void)send(m);
  const auto state = uploader_.state();
  if (state == last_upload_state_) return;
  last_upload_state_ = state;
  if (state == MissionUploader::State::Done) {
    mission_size_ = uploader_.total();
    log("mission uploaded: " + std::to_string(uploader_.total()) + " items");
  } else if (state == MissionUploader::State::Failed) {
    log("mission upload failed: " + uploader_.error());
  }
}

void Station::periodic() {
  stats_.tick(now_us_);

  if (vehicle_.seen && was_online_ && !vehicle_online()) {
    was_online_ = false;
    log("link lost");
  } else if (!was_online_ && vehicle_online()) {
    was_online_ = true;
    log("link restored");
  }

  if (!sent_first_ || now_us_ - last_heartbeat_tx_us_ >= kHeartbeatPeriodUs) {
    Heartbeat hb;
    hb.type = static_cast<std::uint8_t>(MavType::Gcs);
    hb.autopilot = static_cast<std::uint8_t>(MavAutopilot::Invalid);
    hb.system_status = static_cast<std::uint8_t>(MavState::Active);
    (void)send(hb);
    last_heartbeat_tx_us_ = now_us_;
    sent_first_ = true;
  }
  if (vehicle_.seen && now_us_ - last_ping_tx_us_ >= kPingPeriodUs) {
    Ping ping;
    ping.time_usec = now_us_;
    ping.seq = ping_seq_++;
    (void)send(ping);  // target 0/0 = any system
    last_ping_tx_us_ = now_us_;
  }

  relay_uploader(uploader_.poll(now_us_));

  if (command_status_ == CommandStatus::Pending && now_us_ - command_tx_us_ >= kCommandRetryUs) {
    if (command_attempts_ >= kCommandAttempts) {
      command_status_ = CommandStatus::TimedOut;
      log(pending_label_ + ": no response");
    } else {
      pending_command_.confirmation = static_cast<std::uint8_t>(command_attempts_);
      (void)send(pending_command_);
      ++command_attempts_;
      command_tx_us_ = now_us_;
    }
  }
}

bool Station::send_command(CommandLong command, const char* label) {
  if (link_.is_replay()) {
    log("replay mode: commands are disabled");
    return false;
  }
  if (!vehicle_.seen) {
    log(std::string(label) + ": no vehicle connected");
    return false;
  }
  if (command_status_ == CommandStatus::Pending) {
    log(std::string(label) + ": previous command still pending");
    return false;
  }
  command.target_system = vehicle_.sysid;
  command.target_component = vehicle_.compid;
  if (!send(command)) {
    log(std::string(label) + ": send failed");
    return false;
  }
  pending_command_ = command;
  pending_label_ = label;
  command_status_ = CommandStatus::Pending;
  command_tx_us_ = now_us_;
  command_attempts_ = 1;
  log(std::string(label) + " sent");
  return true;
}

bool Station::arm() {
  CommandLong c;
  c.command = static_cast<std::uint16_t>(MavCmd::ComponentArmDisarm);
  c.param1 = 1.0F;
  return send_command(c, "ARM");
}

bool Station::disarm() {
  CommandLong c;
  c.command = static_cast<std::uint16_t>(MavCmd::ComponentArmDisarm);
  c.param1 = 0.0F;
  return send_command(c, "DISARM");
}

bool Station::takeoff(double altitude_m) {
  if (!vehicle_.position) {
    log("TAKEOFF: no position yet");
    return false;
  }
  // MAV_CMD_NAV_TAKEOFF carries an AMSL altitude; derive the ground level from telemetry.
  const double ground_amsl = vehicle_.position->alt_msl - vehicle_.position->alt_rel;
  CommandLong c;
  c.command = static_cast<std::uint16_t>(MavCmd::NavTakeoff);
  c.param1 = kNaN;
  c.param4 = kNaN;
  c.param5 = kNaN;
  c.param6 = kNaN;
  c.param7 = static_cast<float>(ground_amsl + altitude_m);
  return send_command(c, "TAKEOFF");
}

bool Station::land() {
  CommandLong c;
  c.command = static_cast<std::uint16_t>(MavCmd::NavLand);
  c.param4 = kNaN;
  c.param5 = kNaN;
  c.param6 = kNaN;
  return send_command(c, "LAND");
}

bool Station::return_to_launch() {
  CommandLong c;
  c.command = static_cast<std::uint16_t>(MavCmd::NavReturnToLaunch);
  return send_command(c, "RTL");
}

bool Station::start_mission() {
  CommandLong c;
  c.command = static_cast<std::uint16_t>(MavCmd::MissionStart);
  return send_command(c, "MISSION_START");
}

bool Station::upload_mission(std::vector<MissionItem> items) {
  if (link_.is_replay()) {
    log("replay mode: commands are disabled");
    return false;
  }
  if (!vehicle_.seen) {
    log("mission upload: no vehicle connected");
    return false;
  }
  if (uploader_.state() == MissionUploader::State::Uploading) {
    log("mission upload already in progress");
    return false;
  }
  log("uploading mission: " + std::to_string(items.size()) + " items");
  const auto out = uploader_.start(std::move(items), now_us_);
  last_upload_state_ = MissionUploader::State::Uploading;
  relay_uploader(out);
  return uploader_.state() == MissionUploader::State::Uploading;
}

}  // namespace mavgs
