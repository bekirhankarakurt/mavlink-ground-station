#pragma once

#include <deque>

#include "mavgs/link.hpp"
#include "mavgs/sim.hpp"
#include "mavgs/station.hpp"

namespace testing_support {

/// In-memory Link: whatever the station sends is collected, and queued
/// datagrams are handed back on receive().
class LoopLink final : public mavgs::Link {
 public:
  bool send(const mavgs::Bytes& datagram) override {
    sent.push_back(datagram);
    return true;
  }
  bool receive(mavgs::Bytes& out, std::chrono::milliseconds) override {
    if (inbox.empty()) return false;
    out = std::move(inbox.front());
    inbox.pop_front();
    return true;
  }
  std::string describe() const override { return "loop"; }

  std::deque<mavgs::Bytes> inbox;
  std::vector<mavgs::Bytes> sent;
};

/// Station wired to a VehicleSim over a LoopLink with a simulated clock.
struct Harness {
  explicit Harness(mavgs::SimConfig config = {}) : station(link, {}), sim(config) {}

  void run(double seconds, std::uint64_t step_us = 20'000) {
    const auto end = now_us + static_cast<std::uint64_t>(seconds * 1e6);
    while (now_us < end) {
      now_us += step_us;
      station.service(now_us, std::chrono::milliseconds(0));
      for (const auto& datagram : link.sent) {
        for (auto& reply : sim.handle(datagram)) link.inbox.push_back(std::move(reply));
      }
      link.sent.clear();
      for (auto& frame : sim.step(now_us)) link.inbox.push_back(std::move(frame));
    }
  }

  /// Runs until `done()` is true or `timeout_s` of simulated time has passed.
  template <typename Pred>
  bool run_until(Pred done, double timeout_s) {
    const auto end = now_us + static_cast<std::uint64_t>(timeout_s * 1e6);
    while (now_us < end) {
      run(0.02);
      if (done()) return true;
    }
    return false;
  }

  LoopLink link;
  mavgs::Station station;
  mavgs::VehicleSim sim;
  std::uint64_t now_us = 1'000'000;
};

}  // namespace testing_support
