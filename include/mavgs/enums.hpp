#pragma once

#include <cstdint>

namespace mavgs {

enum class MavType : std::uint8_t { Generic = 0, FixedWing = 1, Quadrotor = 2, Gcs = 6 };

enum class MavAutopilot : std::uint8_t { Generic = 0, Ardupilot = 3, Px4 = 12, Invalid = 8 };

enum class MavState : std::uint8_t {
  Uninit = 0,
  Boot = 1,
  Calibrating = 2,
  Standby = 3,
  Active = 4,
  Critical = 5,
  Emergency = 6,
  Poweroff = 7,
  Terminating = 8
};

namespace base_mode {
inline constexpr std::uint8_t kCustomEnabled = 0x01;
inline constexpr std::uint8_t kGuidedEnabled = 0x08;
inline constexpr std::uint8_t kSafetyArmed = 0x80;
}  // namespace base_mode

enum class MavCmd : std::uint16_t {
  NavReturnToLaunch = 20,
  NavLand = 21,
  NavTakeoff = 22,
  NavWaypoint = 16,
  MissionStart = 300,
  ComponentArmDisarm = 400
};

enum class MavResult : std::uint8_t {
  Accepted = 0,
  TemporarilyRejected = 1,
  Denied = 2,
  Unsupported = 3,
  Failed = 4,
  InProgress = 5,
  Cancelled = 6
};

enum class MavFrame : std::uint8_t { Mission = 2, GlobalRelativeAltInt = 6 };

enum class MissionResult : std::uint8_t { Accepted = 0, Error = 1, InvalidSequence = 13 };

/// PX4 custom_mode packing: main mode in byte 2, sub mode in byte 3.
namespace px4 {
inline constexpr std::uint8_t kMainManual = 1;
inline constexpr std::uint8_t kMainAltctl = 2;
inline constexpr std::uint8_t kMainPosctl = 3;
inline constexpr std::uint8_t kMainAuto = 4;
inline constexpr std::uint8_t kSubReady = 1;
inline constexpr std::uint8_t kSubTakeoff = 2;
inline constexpr std::uint8_t kSubLoiter = 3;
inline constexpr std::uint8_t kSubMission = 4;
inline constexpr std::uint8_t kSubRtl = 5;
inline constexpr std::uint8_t kSubLand = 6;
[[nodiscard]] constexpr std::uint32_t custom_mode(std::uint8_t main, std::uint8_t sub) {
  return (static_cast<std::uint32_t>(main) << 16) | (static_cast<std::uint32_t>(sub) << 24);
}
}  // namespace px4

}  // namespace mavgs
