#include "mavgs/vehicle_state.hpp"

#include <cmath>

#include "mavgs/enums.hpp"

namespace mavgs {

namespace {

constexpr double kRadToDeg = 57.29577951308232;

std::string px4_mode(std::uint32_t custom_mode) {
  const auto main = static_cast<std::uint8_t>((custom_mode >> 16) & 0xFFu);
  const auto sub = static_cast<std::uint8_t>((custom_mode >> 24) & 0xFFu);
  switch (main) {
    case px4::kMainManual: return "MANUAL";
    case px4::kMainAltctl: return "ALTCTL";
    case px4::kMainPosctl: return "POSCTL";
    case px4::kMainAuto:
      switch (sub) {
        case px4::kSubReady: return "AUTO.READY";
        case px4::kSubTakeoff: return "AUTO.TAKEOFF";
        case px4::kSubLoiter: return "AUTO.LOITER";
        case px4::kSubMission: return "AUTO.MISSION";
        case px4::kSubRtl: return "AUTO.RTL";
        case px4::kSubLand: return "AUTO.LAND";
        default: return "AUTO";
      }
    default: return "MODE " + std::to_string(main);
  }
}

std::string ardupilot_copter_mode(std::uint32_t custom_mode) {
  switch (custom_mode) {
    case 0: return "STABILIZE";
    case 1: return "ACRO";
    case 2: return "ALT_HOLD";
    case 3: return "AUTO";
    case 4: return "GUIDED";
    case 5: return "LOITER";
    case 6: return "RTL";
    case 9: return "LAND";
    case 16: return "POSHOLD";
    default: return "MODE " + std::to_string(custom_mode);
  }
}

}  // namespace

std::string VehicleState::mode_name() const {
  if (!seen) return "-";
  if ((base_mode & base_mode::kCustomEnabled) == 0) return "MODE 0x" + std::to_string(base_mode);
  if (autopilot == static_cast<std::uint8_t>(MavAutopilot::Px4)) return px4_mode(custom_mode);
  if (autopilot == static_cast<std::uint8_t>(MavAutopilot::Ardupilot)) {
    return ardupilot_copter_mode(custom_mode);
  }
  return "MODE " + std::to_string(custom_mode);
}

std::string VehicleState::status_name() const {
  switch (static_cast<MavState>(system_status)) {
    case MavState::Uninit: return "UNINIT";
    case MavState::Boot: return "BOOT";
    case MavState::Calibrating: return "CALIBRATING";
    case MavState::Standby: return "STANDBY";
    case MavState::Active: return "ACTIVE";
    case MavState::Critical: return "CRITICAL";
    case MavState::Emergency: return "EMERGENCY";
    case MavState::Poweroff: return "POWEROFF";
    case MavState::Terminating: return "TERMINATING";
  }
  return "?";
}

void VehicleState::apply(const Message& message, std::uint64_t now_us) {
  if (const auto* hb = std::get_if<Heartbeat>(&message)) {
    seen = true;
    type = hb->type;
    autopilot = hb->autopilot;
    base_mode = hb->base_mode;
    system_status = hb->system_status;
    custom_mode = hb->custom_mode;
    last_heartbeat_us = now_us;
  } else if (const auto* ss = std::get_if<SysStatus>(&message)) {
    Battery b;
    b.volts = ss->voltage_battery == 0xFFFF ? 0.0 : ss->voltage_battery / 1000.0;
    b.amps = ss->current_battery < 0 ? 0.0 : ss->current_battery / 100.0;
    b.percent = ss->battery_remaining;
    battery = b;
  } else if (const auto* att = std::get_if<Attitude>(&message)) {
    attitude = Orientation{att->roll * kRadToDeg, att->pitch * kRadToDeg, att->yaw * kRadToDeg};
  } else if (const auto* pos = std::get_if<GlobalPositionInt>(&message)) {
    Position p;
    p.lat = pos->lat / 1e7;
    p.lon = pos->lon / 1e7;
    p.alt_msl = pos->alt / 1000.0;
    p.alt_rel = pos->relative_alt / 1000.0;
    p.hdg_deg = pos->hdg == 0xFFFF ? 0.0 : pos->hdg / 100.0;
    position = p;
  } else if (const auto* raw = std::get_if<GpsRawInt>(&message)) {
    gps = Gps{raw->fix_type, raw->satellites_visible, raw->eph == 0xFFFF ? 0.0 : raw->eph / 100.0};
  } else if (const auto* hud_msg = std::get_if<VfrHud>(&message)) {
    hud = Hud{hud_msg->airspeed, hud_msg->groundspeed, hud_msg->climb, hud_msg->throttle};
  } else if (const auto* cur = std::get_if<MissionCurrent>(&message)) {
    mission_current = cur->seq;
  }
}

}  // namespace mavgs
