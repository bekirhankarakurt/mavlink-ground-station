#pragma once

#include <cstddef>
#include <cstdint>

namespace mavgs {

inline constexpr std::uint16_t kCrcInit = 0xFFFF;

/// X.25 CRC accumulate step (MCRF4XX), as used by MAVLink.
[[nodiscard]] constexpr std::uint16_t crc_accumulate(std::uint8_t byte, std::uint16_t crc) {
  auto tmp = static_cast<std::uint8_t>(byte ^ (crc & 0xFFu));
  tmp = static_cast<std::uint8_t>(tmp ^ static_cast<std::uint8_t>(tmp << 4));
  return static_cast<std::uint16_t>((crc >> 8) ^ (static_cast<unsigned>(tmp) << 8) ^
                                    (static_cast<unsigned>(tmp) << 3) ^ (tmp >> 4));
}

[[nodiscard]] inline std::uint16_t crc_calculate(const std::uint8_t* data, std::size_t size,
                                                 std::uint16_t crc = kCrcInit) {
  for (std::size_t i = 0; i < size; ++i) crc = crc_accumulate(data[i], crc);
  return crc;
}

}  // namespace mavgs
