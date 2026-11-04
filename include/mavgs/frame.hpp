#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "mavgs/bytes.hpp"

namespace mavgs {

inline constexpr std::uint8_t kMagicV2 = 0xFD;
inline constexpr std::size_t kHeaderSize = 10;
inline constexpr std::size_t kChecksumSize = 2;
inline constexpr std::size_t kSignatureSize = 13;
inline constexpr std::size_t kMaxPayload = 255;
inline constexpr std::uint8_t kIncompatSigned = 0x01;

/// One MAVLink v2 frame. `payload` is stored as on the wire (zero-truncated).
struct Frame {
  std::uint8_t seq = 0;
  std::uint8_t sysid = 0;
  std::uint8_t compid = 0;
  std::uint32_t msgid = 0;
  Bytes payload;
};

/// CRC_EXTRA seed for a message id of the common dialect, if known.
[[nodiscard]] std::optional<std::uint8_t> crc_extra(std::uint32_t msgid);

/// Serializes a frame, truncating trailing zero payload bytes (keeping at
/// least one). Throws std::invalid_argument for an unknown message id or an
/// oversized payload.
[[nodiscard]] Bytes serialize(const Frame& frame);

/// Total wire length of the v2 frame starting at `data`, or nullopt if fewer
/// than the 3 header bytes needed to tell are available or the magic is wrong.
[[nodiscard]] std::optional<std::size_t> frame_length(const std::uint8_t* data, std::size_t avail);

struct ParsedFrame {
  Frame frame;
  Bytes raw;            ///< exact wire bytes including checksum and signature
  bool verified = true; ///< false when the id is unknown so the CRC cannot be checked
};

struct ParserStats {
  std::uint64_t frames = 0;
  std::uint64_t crc_errors = 0;
  std::uint64_t bad_flags = 0;
  std::uint64_t discarded_bytes = 0;
};

/// Incremental MAVLink v2 parser. Resynchronizes on the next magic byte after
/// corruption. Signatures are skipped, not verified.
class Parser {
 public:
  [[nodiscard]] std::vector<ParsedFrame> feed(const std::uint8_t* data, std::size_t size);
  [[nodiscard]] std::vector<ParsedFrame> feed(const Bytes& bytes) {
    return feed(bytes.data(), bytes.size());
  }
  [[nodiscard]] const ParserStats& stats() const { return stats_; }

 private:
  Bytes buffer_;
  ParserStats stats_;
};

}  // namespace mavgs
