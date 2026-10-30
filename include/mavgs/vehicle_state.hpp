#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "mavgs/messages.hpp"

namespace mavgs {

/// Latest telemetry for one vehicle, in display units (degrees, metres, volts).
struct VehicleState {
  struct Battery {
    double volts = 0;
    double amps = 0;
    int percent = -1;  ///< -1 when unknown
  };
  struct Position {
    double lat = 0, lon = 0;  ///< degrees
    double alt_msl = 0;       ///< m
    double alt_rel = 0;       ///< m above home
    double hdg_deg = 0;
  };
  struct Orientation {
    double roll = 0, pitch = 0, yaw = 0;  ///< degrees
  };
  struct Gps {
    int fix_type = 0;
    int satellites = 0;
    double hdop = 0;
  };
  struct Hud {
    double airspeed = 0, groundspeed = 0, climb = 0;  ///< m/s
    int throttle = 0;                                  ///< %
  };

  bool seen = false;  ///< at least one heartbeat received
  std::uint8_t sysid = 0;
  std::uint8_t compid = 0;
  std::uint8_t type = 0;
  std::uint8_t autopilot = 0;
  std::uint8_t base_mode = 0;
  std::uint8_t system_status = 0;
  std::uint32_t custom_mode = 0;
  std::uint64_t last_heartbeat_us = 0;

  std::optional<Battery> battery;
  std::optional<Position> position;
  std::optional<Orientation> attitude;
  std::optional<Gps> gps;
  std::optional<Hud> hud;
  std::optional<std::uint16_t> mission_current;

  [[nodiscard]] bool armed() const { return (base_mode & 0x80) != 0; }
  [[nodiscard]] std::string mode_name() const;
  [[nodiscard]] std::string status_name() const;

  /// Folds a message from the tracked vehicle into the state.
  void apply(const Message& message, std::uint64_t now_us);
};

}  // namespace mavgs
