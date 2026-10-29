#include "mavgs/messages.hpp"

namespace mavgs {

namespace {
constexpr std::size_t kStatusTextSize = 50;
}

// Wire order = fields sorted by type size (largest first), declaration order within a size.

void Heartbeat::write(Writer& w) const {
  w.put(custom_mode);
  w.put(type);
  w.put(autopilot);
  w.put(base_mode);
  w.put(system_status);
  w.put(mavlink_version);
}
Heartbeat Heartbeat::read(Reader& r) {
  Heartbeat m;
  m.custom_mode = r.get<std::uint32_t>();
  m.type = r.get<std::uint8_t>();
  m.autopilot = r.get<std::uint8_t>();
  m.base_mode = r.get<std::uint8_t>();
  m.system_status = r.get<std::uint8_t>();
  m.mavlink_version = r.get<std::uint8_t>();
  return m;
}

void SysStatus::write(Writer& w) const {
  w.put(sensors_present);
  w.put(sensors_enabled);
  w.put(sensors_health);
  w.put(load);
  w.put(voltage_battery);
  w.put(current_battery);
  w.put(drop_rate_comm);
  for (int i = 0; i < 5; ++i) w.put(std::uint16_t{0});  // errors_comm, errors_count1..4
  w.put(battery_remaining);
}
SysStatus SysStatus::read(Reader& r) {
  SysStatus m;
  m.sensors_present = r.get<std::uint32_t>();
  m.sensors_enabled = r.get<std::uint32_t>();
  m.sensors_health = r.get<std::uint32_t>();
  m.load = r.get<std::uint16_t>();
  m.voltage_battery = r.get<std::uint16_t>();
  m.current_battery = r.get<std::int16_t>();
  m.drop_rate_comm = r.get<std::uint16_t>();
  for (int i = 0; i < 5; ++i) (void)r.get<std::uint16_t>();
  m.battery_remaining = r.get<std::int8_t>();
  return m;
}

void Attitude::write(Writer& w) const {
  w.put(time_boot_ms);
  w.put(roll);
  w.put(pitch);
  w.put(yaw);
  w.put(rollspeed);
  w.put(pitchspeed);
  w.put(yawspeed);
}
Attitude Attitude::read(Reader& r) {
  Attitude m;
  m.time_boot_ms = r.get<std::uint32_t>();
  m.roll = r.get_float();
  m.pitch = r.get_float();
  m.yaw = r.get_float();
  m.rollspeed = r.get_float();
  m.pitchspeed = r.get_float();
  m.yawspeed = r.get_float();
  return m;
}

void GlobalPositionInt::write(Writer& w) const {
  w.put(time_boot_ms);
  w.put(lat);
  w.put(lon);
  w.put(alt);
  w.put(relative_alt);
  w.put(vx);
  w.put(vy);
  w.put(vz);
  w.put(hdg);
}
GlobalPositionInt GlobalPositionInt::read(Reader& r) {
  GlobalPositionInt m;
  m.time_boot_ms = r.get<std::uint32_t>();
  m.lat = r.get<std::int32_t>();
  m.lon = r.get<std::int32_t>();
  m.alt = r.get<std::int32_t>();
  m.relative_alt = r.get<std::int32_t>();
  m.vx = r.get<std::int16_t>();
  m.vy = r.get<std::int16_t>();
  m.vz = r.get<std::int16_t>();
  m.hdg = r.get<std::uint16_t>();
  return m;
}

void GpsRawInt::write(Writer& w) const {
  w.put(time_usec);
  w.put(lat);
  w.put(lon);
  w.put(alt);
  w.put(eph);
  w.put(epv);
  w.put(vel);
  w.put(cog);
  w.put(fix_type);
  w.put(satellites_visible);
}
GpsRawInt GpsRawInt::read(Reader& r) {
  GpsRawInt m;
  m.time_usec = r.get<std::uint64_t>();
  m.lat = r.get<std::int32_t>();
  m.lon = r.get<std::int32_t>();
  m.alt = r.get<std::int32_t>();
  m.eph = r.get<std::uint16_t>();
  m.epv = r.get<std::uint16_t>();
  m.vel = r.get<std::uint16_t>();
  m.cog = r.get<std::uint16_t>();
  m.fix_type = r.get<std::uint8_t>();
  m.satellites_visible = r.get<std::uint8_t>();
  return m;
}

void VfrHud::write(Writer& w) const {
  w.put(airspeed);
  w.put(groundspeed);
  w.put(alt);
  w.put(climb);
  w.put(heading);
  w.put(throttle);
}
VfrHud VfrHud::read(Reader& r) {
  VfrHud m;
  m.airspeed = r.get_float();
  m.groundspeed = r.get_float();
  m.alt = r.get_float();
  m.climb = r.get_float();
  m.heading = r.get<std::int16_t>();
  m.throttle = r.get<std::uint16_t>();
  return m;
}

void CommandLong::write(Writer& w) const {
  w.put(param1);
  w.put(param2);
  w.put(param3);
  w.put(param4);
  w.put(param5);
  w.put(param6);
  w.put(param7);
  w.put(command);
  w.put(target_system);
  w.put(target_component);
  w.put(confirmation);
}
CommandLong CommandLong::read(Reader& r) {
  CommandLong m;
  m.param1 = r.get_float();
  m.param2 = r.get_float();
  m.param3 = r.get_float();
  m.param4 = r.get_float();
  m.param5 = r.get_float();
  m.param6 = r.get_float();
  m.param7 = r.get_float();
  m.command = r.get<std::uint16_t>();
  m.target_system = r.get<std::uint8_t>();
  m.target_component = r.get<std::uint8_t>();
  m.confirmation = r.get<std::uint8_t>();
  return m;
}

void CommandAck::write(Writer& w) const {
  w.put(command);
  w.put(result);
}
CommandAck CommandAck::read(Reader& r) {
  CommandAck m;
  m.command = r.get<std::uint16_t>();
  m.result = r.get<std::uint8_t>();
  return m;
}

void MissionCount::write(Writer& w) const {
  w.put(count);
  w.put(target_system);
  w.put(target_component);
}
MissionCount MissionCount::read(Reader& r) {
  MissionCount m;
  m.count = r.get<std::uint16_t>();
  m.target_system = r.get<std::uint8_t>();
  m.target_component = r.get<std::uint8_t>();
  return m;
}

void MissionRequestInt::write(Writer& w) const {
  w.put(seq);
  w.put(target_system);
  w.put(target_component);
}
MissionRequestInt MissionRequestInt::read(Reader& r) {
  MissionRequestInt m;
  m.seq = r.get<std::uint16_t>();
  m.target_system = r.get<std::uint8_t>();
  m.target_component = r.get<std::uint8_t>();
  return m;
}

void MissionItemInt::write(Writer& w) const {
  w.put(param1);
  w.put(param2);
  w.put(param3);
  w.put(param4);
  w.put(x);
  w.put(y);
  w.put(z);
  w.put(seq);
  w.put(command);
  w.put(target_system);
  w.put(target_component);
  w.put(frame);
  w.put(current);
  w.put(autocontinue);
}
MissionItemInt MissionItemInt::read(Reader& r) {
  MissionItemInt m;
  m.param1 = r.get_float();
  m.param2 = r.get_float();
  m.param3 = r.get_float();
  m.param4 = r.get_float();
  m.x = r.get<std::int32_t>();
  m.y = r.get<std::int32_t>();
  m.z = r.get_float();
  m.seq = r.get<std::uint16_t>();
  m.command = r.get<std::uint16_t>();
  m.target_system = r.get<std::uint8_t>();
  m.target_component = r.get<std::uint8_t>();
  m.frame = r.get<std::uint8_t>();
  m.current = r.get<std::uint8_t>();
  m.autocontinue = r.get<std::uint8_t>();
  return m;
}

void MissionAck::write(Writer& w) const {
  w.put(target_system);
  w.put(target_component);
  w.put(type);
}
MissionAck MissionAck::read(Reader& r) {
  MissionAck m;
  m.target_system = r.get<std::uint8_t>();
  m.target_component = r.get<std::uint8_t>();
  m.type = r.get<std::uint8_t>();
  return m;
}

void MissionCurrent::write(Writer& w) const { w.put(seq); }
MissionCurrent MissionCurrent::read(Reader& r) {
  MissionCurrent m;
  m.seq = r.get<std::uint16_t>();
  return m;
}

void StatusText::write(Writer& w) const {
  w.put(severity);
  w.put_chars(text, kStatusTextSize);
}
StatusText StatusText::read(Reader& r) {
  StatusText m;
  m.severity = r.get<std::uint8_t>();
  m.text = r.get_chars(kStatusTextSize);
  return m;
}

void Ping::write(Writer& w) const {
  w.put(time_usec);
  w.put(seq);
  w.put(target_system);
  w.put(target_component);
}
Ping Ping::read(Reader& r) {
  Ping m;
  m.time_usec = r.get<std::uint64_t>();
  m.seq = r.get<std::uint32_t>();
  m.target_system = r.get<std::uint8_t>();
  m.target_component = r.get<std::uint8_t>();
  return m;
}

std::uint32_t message_id(const Message& message) {
  return std::visit([](const auto& m) { return std::decay_t<decltype(m)>::kId; }, message);
}

Bytes encode_payload(const Message& message) {
  Writer w;
  std::visit([&w](const auto& m) { m.write(w); }, message);
  return w.take();
}

namespace {

template <typename M>
bool try_decode(const Frame& frame, std::optional<Message>& out) {
  const bool match =
      frame.msgid == M::kId ||
      (std::is_same_v<M, MissionRequestInt> && frame.msgid == MissionRequestInt::kLegacyId);
  if (!match) return false;
  Reader r(frame.payload);
  out = M::read(r);
  return true;
}

template <typename... Ms>
std::optional<Message> decode_any(const Frame& frame, std::variant<Ms...>*) {
  std::optional<Message> out;
  (void)(try_decode<Ms>(frame, out) || ...);
  return out;
}

}  // namespace

std::optional<Message> decode(const Frame& frame) {
  return decode_any(frame, static_cast<Message*>(nullptr));
}

Bytes Packer::pack(const Message& message) {
  Frame frame;
  frame.seq = seq_++;
  frame.sysid = sysid_;
  frame.compid = compid_;
  frame.msgid = message_id(message);
  frame.payload = encode_payload(message);
  return serialize(frame);
}

}  // namespace mavgs
