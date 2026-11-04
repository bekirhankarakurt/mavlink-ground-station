#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "mavgs/bytes.hpp"
#include "mavgs/frame.hpp"

namespace mavgs {

// Field order inside each struct follows the declaration order of the MAVLink
// XML; wire order (sorted by type size) is handled in write()/read().

struct Heartbeat {
  static constexpr std::uint32_t kId = 0;
  std::uint32_t custom_mode = 0;
  std::uint8_t type = 0;
  std::uint8_t autopilot = 0;
  std::uint8_t base_mode = 0;
  std::uint8_t system_status = 0;
  std::uint8_t mavlink_version = 3;
  void write(Writer& w) const;
  [[nodiscard]] static Heartbeat read(Reader& r);
};

struct SysStatus {
  static constexpr std::uint32_t kId = 1;
  std::uint32_t sensors_present = 0;
  std::uint32_t sensors_enabled = 0;
  std::uint32_t sensors_health = 0;
  std::uint16_t load = 0;              ///< 0..1000 (d%)
  std::uint16_t voltage_battery = 0;   ///< mV
  std::int16_t current_battery = -1;   ///< cA (10 mA), -1 if unknown
  std::int8_t battery_remaining = -1;  ///< %, -1 if unknown
  std::uint16_t drop_rate_comm = 0;    ///< c%
  void write(Writer& w) const;
  [[nodiscard]] static SysStatus read(Reader& r);
};

struct Attitude {
  static constexpr std::uint32_t kId = 30;
  std::uint32_t time_boot_ms = 0;
  float roll = 0, pitch = 0, yaw = 0;  ///< rad
  float rollspeed = 0, pitchspeed = 0, yawspeed = 0;
  void write(Writer& w) const;
  [[nodiscard]] static Attitude read(Reader& r);
};

struct GlobalPositionInt {
  static constexpr std::uint32_t kId = 33;
  std::uint32_t time_boot_ms = 0;
  std::int32_t lat = 0, lon = 0;  ///< degE7
  std::int32_t alt = 0;           ///< mm AMSL
  std::int32_t relative_alt = 0;  ///< mm above home
  std::int16_t vx = 0, vy = 0, vz = 0;  ///< cm/s NED
  std::uint16_t hdg = 0;                ///< cdeg
  void write(Writer& w) const;
  [[nodiscard]] static GlobalPositionInt read(Reader& r);
};

struct GpsRawInt {
  static constexpr std::uint32_t kId = 24;
  std::uint64_t time_usec = 0;
  std::int32_t lat = 0, lon = 0, alt = 0;
  std::uint16_t eph = 0xFFFF, epv = 0xFFFF;  ///< HDOP/VDOP x100
  std::uint16_t vel = 0;                     ///< cm/s
  std::uint16_t cog = 0;                     ///< cdeg
  std::uint8_t fix_type = 0;
  std::uint8_t satellites_visible = 0;
  void write(Writer& w) const;
  [[nodiscard]] static GpsRawInt read(Reader& r);
};

struct VfrHud {
  static constexpr std::uint32_t kId = 74;
  float airspeed = 0, groundspeed = 0, alt = 0, climb = 0;
  std::int16_t heading = 0;
  std::uint16_t throttle = 0;
  void write(Writer& w) const;
  [[nodiscard]] static VfrHud read(Reader& r);
};

struct CommandLong {
  static constexpr std::uint32_t kId = 76;
  float param1 = 0, param2 = 0, param3 = 0, param4 = 0, param5 = 0, param6 = 0, param7 = 0;
  std::uint16_t command = 0;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
  std::uint8_t confirmation = 0;
  void write(Writer& w) const;
  [[nodiscard]] static CommandLong read(Reader& r);
};

struct CommandAck {
  static constexpr std::uint32_t kId = 77;
  std::uint16_t command = 0;
  std::uint8_t result = 0;
  void write(Writer& w) const;
  [[nodiscard]] static CommandAck read(Reader& r);
};

struct MissionCount {
  static constexpr std::uint32_t kId = 44;
  std::uint16_t count = 0;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
  void write(Writer& w) const;
  [[nodiscard]] static MissionCount read(Reader& r);
};

/// Used for both MISSION_REQUEST (40) and MISSION_REQUEST_INT (51), which share a layout.
struct MissionRequestInt {
  static constexpr std::uint32_t kId = 51;
  static constexpr std::uint32_t kLegacyId = 40;
  std::uint16_t seq = 0;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
  void write(Writer& w) const;
  [[nodiscard]] static MissionRequestInt read(Reader& r);
};

struct MissionItemInt {
  static constexpr std::uint32_t kId = 73;
  float param1 = 0, param2 = 0, param3 = 0, param4 = 0;
  std::int32_t x = 0, y = 0;  ///< degE7
  float z = 0;                ///< m
  std::uint16_t seq = 0;
  std::uint16_t command = 0;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
  std::uint8_t frame = 0;
  std::uint8_t current = 0;
  std::uint8_t autocontinue = 1;
  void write(Writer& w) const;
  [[nodiscard]] static MissionItemInt read(Reader& r);
};

struct MissionAck {
  static constexpr std::uint32_t kId = 47;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
  std::uint8_t type = 0;
  void write(Writer& w) const;
  [[nodiscard]] static MissionAck read(Reader& r);
};

struct MissionCurrent {
  static constexpr std::uint32_t kId = 42;
  std::uint16_t seq = 0;
  void write(Writer& w) const;
  [[nodiscard]] static MissionCurrent read(Reader& r);
};

struct StatusText {
  static constexpr std::uint32_t kId = 253;
  std::uint8_t severity = 6;  ///< MAV_SEVERITY_INFO
  std::string text;           ///< at most 50 chars on the wire
  void write(Writer& w) const;
  [[nodiscard]] static StatusText read(Reader& r);
};

struct Ping {
  static constexpr std::uint32_t kId = 4;
  std::uint64_t time_usec = 0;
  std::uint32_t seq = 0;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
  void write(Writer& w) const;
  [[nodiscard]] static Ping read(Reader& r);
};

using Message =
    std::variant<Heartbeat, SysStatus, Attitude, GlobalPositionInt, GpsRawInt, VfrHud, CommandLong,
                 CommandAck, MissionCount, MissionRequestInt, MissionItemInt, MissionAck,
                 MissionCurrent, StatusText, Ping>;

[[nodiscard]] std::uint32_t message_id(const Message& message);
[[nodiscard]] Bytes encode_payload(const Message& message);

/// Decodes a frame into a typed message; nullopt for ids outside the supported set.
[[nodiscard]] std::optional<Message> decode(const Frame& frame);

/// Builds frames with a rolling sequence number for one sender identity.
class Packer {
 public:
  Packer(std::uint8_t sysid, std::uint8_t compid) : sysid_(sysid), compid_(compid) {}
  [[nodiscard]] Bytes pack(const Message& message);

 private:
  std::uint8_t sysid_;
  std::uint8_t compid_;
  std::uint8_t seq_ = 0;
};

}  // namespace mavgs
