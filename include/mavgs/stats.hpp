#pragma once

#include <cstdint>
#include <map>

namespace mavgs {

/// Link-quality accounting: packet loss from MAVLink sequence gaps, message and
/// byte rates, and ping round-trip latency. All times are caller-supplied
/// microseconds so the logic is deterministic under test.
class LinkStats {
 public:
  /// Records one received frame from (sysid, compid) carrying sequence `seq`.
  void on_frame(std::uint8_t sysid, std::uint8_t compid, std::uint8_t seq, std::size_t bytes);
  void on_crc_errors(std::uint64_t total) { crc_errors_ = total; }
  void on_latency_sample(std::uint64_t rtt_us);
  /// Refreshes the rate figures; call regularly. Rates update once per second.
  void tick(std::uint64_t now_us);

  [[nodiscard]] std::uint64_t received() const { return received_; }
  [[nodiscard]] std::uint64_t lost() const { return lost_; }
  [[nodiscard]] std::uint64_t bytes() const { return bytes_; }
  [[nodiscard]] std::uint64_t crc_errors() const { return crc_errors_; }
  [[nodiscard]] double loss_percent() const;
  [[nodiscard]] double msg_rate() const { return msg_rate_; }
  [[nodiscard]] double byte_rate() const { return byte_rate_; }

  [[nodiscard]] bool has_latency() const { return samples_ > 0; }
  [[nodiscard]] double latency_last_ms() const { return static_cast<double>(last_us_) / 1000.0; }
  [[nodiscard]] double latency_avg_ms() const;
  [[nodiscard]] double latency_min_ms() const { return static_cast<double>(min_us_) / 1000.0; }
  [[nodiscard]] double latency_max_ms() const { return static_cast<double>(max_us_) / 1000.0; }

 private:
  std::map<std::uint16_t, std::uint8_t> last_seq_;
  std::uint64_t received_ = 0;
  std::uint64_t lost_ = 0;
  std::uint64_t bytes_ = 0;
  std::uint64_t crc_errors_ = 0;

  std::uint64_t samples_ = 0;
  std::uint64_t sum_us_ = 0;
  std::uint64_t last_us_ = 0;
  std::uint64_t min_us_ = 0;
  std::uint64_t max_us_ = 0;

  std::uint64_t window_start_us_ = 0;
  std::uint64_t window_received_ = 0;
  std::uint64_t window_bytes_ = 0;
  bool window_open_ = false;
  double msg_rate_ = 0;
  double byte_rate_ = 0;
};

}  // namespace mavgs
