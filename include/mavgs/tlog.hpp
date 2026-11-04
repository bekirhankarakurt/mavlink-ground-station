#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "mavgs/bytes.hpp"

namespace mavgs {

/// One telemetry-log record: big-endian microsecond timestamp followed by the
/// raw MAVLink frame. This is the layout used by QGroundControl and
/// Mission Planner `.tlog` files, so recordings open in those tools.
struct TlogRecord {
  std::uint64_t timestamp_us = 0;
  Bytes frame;
};

class TlogWriter {
 public:
  /// Truncates/creates `path`. Throws std::runtime_error if it cannot be opened.
  explicit TlogWriter(const std::string& path);
  void append(std::uint64_t timestamp_us, const Bytes& frame);
  [[nodiscard]] std::uint64_t records() const { return records_; }

 private:
  std::ofstream out_;
  std::uint64_t records_ = 0;
};

/// Reads every complete record. An incomplete or corrupt tail is ignored.
/// Throws std::runtime_error if the file cannot be opened.
[[nodiscard]] std::vector<TlogRecord> read_tlog(const std::string& path);

/// Same as read_tlog() but over an in-memory buffer.
[[nodiscard]] std::vector<TlogRecord> parse_tlog(const Bytes& data);

}  // namespace mavgs
