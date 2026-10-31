#include "mavgs/frame.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>

#include "crc_extra_table.hpp"
#include "mavgs/crc.hpp"

namespace mavgs {

std::optional<std::uint8_t> crc_extra(std::uint32_t msgid) {
  const auto* begin = std::begin(detail::kCrcExtraTable);
  const auto* end = std::end(detail::kCrcExtraTable);
  const auto* it = std::lower_bound(
      begin, end, msgid,
      [](const detail::CrcExtraEntry& entry, std::uint32_t id) { return entry.msgid < id; });
  if (it == end || it->msgid != msgid) return std::nullopt;
  return it->crc_extra;
}

namespace {

std::uint16_t frame_crc(const std::uint8_t* frame, std::size_t payload_len, std::uint8_t extra) {
  std::uint16_t crc = crc_calculate(frame + 1, kHeaderSize - 1 + payload_len);
  return crc_accumulate(extra, crc);
}

/// Total frame size from the first three header bytes (length, incompat flags).
std::size_t wire_length(const std::uint8_t* frame) {
  const std::size_t signature = (frame[2] & kIncompatSigned) != 0 ? kSignatureSize : 0;
  return kHeaderSize + frame[1] + kChecksumSize + signature;
}

}  // namespace

Bytes serialize(const Frame& frame) {
  const auto extra = crc_extra(frame.msgid);
  if (!extra) throw std::invalid_argument("unknown MAVLink message id");
  if (frame.payload.size() > kMaxPayload) throw std::invalid_argument("payload too large");

  std::size_t len = frame.payload.size();
  while (len > 1 && frame.payload[len - 1] == 0) --len;
  if (frame.payload.empty()) len = 0;

  Bytes out;
  out.reserve(kHeaderSize + len + kChecksumSize);
  out.push_back(kMagicV2);
  out.push_back(static_cast<std::uint8_t>(len));
  out.push_back(0);  // incompat flags
  out.push_back(0);  // compat flags
  out.push_back(frame.seq);
  out.push_back(frame.sysid);
  out.push_back(frame.compid);
  out.push_back(static_cast<std::uint8_t>(frame.msgid & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((frame.msgid >> 8) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((frame.msgid >> 16) & 0xFFU));
  out.insert(out.end(), frame.payload.begin(),
             frame.payload.begin() + static_cast<std::ptrdiff_t>(len));

  const std::uint16_t crc = frame_crc(out.data(), len, *extra);
  out.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  out.push_back(static_cast<std::uint8_t>(crc >> 8));
  return out;
}

std::optional<std::size_t> frame_length(const std::uint8_t* data, std::size_t avail) {
  if (avail < 3 || data[0] != kMagicV2) return std::nullopt;
  return wire_length(data);
}

std::vector<ParsedFrame> Parser::feed(const std::uint8_t* data, std::size_t size) {
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<ParsedFrame> out;
  std::size_t pos = 0;

  while (pos < buffer_.size()) {
    if (buffer_[pos] != kMagicV2) {
      ++pos;
      ++stats_.discarded_bytes;
      continue;
    }
    const std::size_t avail = buffer_.size() - pos;
    if (avail < 3) break;
    const std::uint8_t* p = buffer_.data() + pos;
    if ((p[2] & static_cast<std::uint8_t>(~kIncompatSigned)) != 0) {
      ++stats_.bad_flags;
      ++pos;
      ++stats_.discarded_bytes;
      continue;
    }
    const std::size_t total = wire_length(p);
    if (avail < total) break;

    const std::size_t len = p[1];
    const std::uint32_t msgid = static_cast<std::uint32_t>(p[7]) |
                                (static_cast<std::uint32_t>(p[8]) << 8) |
                                (static_cast<std::uint32_t>(p[9]) << 16);
    const auto extra = crc_extra(msgid);
    if (extra) {
      const std::uint16_t expected = frame_crc(p, len, *extra);
      const auto got = static_cast<std::uint16_t>(
          p[kHeaderSize + len] | (static_cast<unsigned>(p[kHeaderSize + len + 1]) << 8));
      if (expected != got) {
        ++stats_.crc_errors;
        ++pos;
        ++stats_.discarded_bytes;
        continue;
      }
    }

    ParsedFrame parsed;
    parsed.frame.seq = p[4];
    parsed.frame.sysid = p[5];
    parsed.frame.compid = p[6];
    parsed.frame.msgid = msgid;
    parsed.frame.payload.assign(p + kHeaderSize, p + kHeaderSize + len);
    parsed.raw.assign(p, p + total);
    parsed.verified = extra.has_value();
    out.push_back(std::move(parsed));
    ++stats_.frames;
    pos += total;
  }

  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
  return out;
}

}  // namespace mavgs
