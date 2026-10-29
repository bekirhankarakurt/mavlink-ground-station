#include "mavgs/stats.hpp"

#include <algorithm>

namespace mavgs {

void LinkStats::on_frame(std::uint8_t sysid, std::uint8_t compid, std::uint8_t seq,
                         std::size_t bytes) {
  const auto key = static_cast<std::uint16_t>((sysid << 8) | compid);
  const auto it = last_seq_.find(key);
  if (it != last_seq_.end()) {
    const auto delta = static_cast<std::uint8_t>(seq - it->second);
    if (delta > 1) lost_ += static_cast<std::uint64_t>(delta - 1);
  }
  last_seq_[key] = seq;
  ++received_;
  bytes_ += bytes;
}

void LinkStats::on_latency_sample(std::uint64_t rtt_us) {
  last_us_ = rtt_us;
  min_us_ = samples_ == 0 ? rtt_us : std::min(min_us_, rtt_us);
  max_us_ = std::max(max_us_, rtt_us);
  sum_us_ += rtt_us;
  ++samples_;
}

double LinkStats::loss_percent() const {
  const auto total = received_ + lost_;
  return total == 0 ? 0.0 : 100.0 * static_cast<double>(lost_) / static_cast<double>(total);
}

double LinkStats::latency_avg_ms() const {
  return samples_ == 0 ? 0.0 : static_cast<double>(sum_us_) / static_cast<double>(samples_) / 1000.0;
}

void LinkStats::tick(std::uint64_t now_us) {
  if (!window_open_) {
    window_open_ = true;
    window_start_us_ = now_us;
    window_received_ = received_;
    window_bytes_ = bytes_;
    return;
  }
  const auto elapsed = now_us - window_start_us_;
  if (elapsed < 1'000'000) return;
  const double seconds = static_cast<double>(elapsed) / 1e6;
  msg_rate_ = static_cast<double>(received_ - window_received_) / seconds;
  byte_rate_ = static_cast<double>(bytes_ - window_bytes_) / seconds;
  window_start_us_ = now_us;
  window_received_ = received_;
  window_bytes_ = bytes_;
}

}  // namespace mavgs
