#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mavgs/bytes.hpp"
#include "mavgs/tlog.hpp"

namespace mavgs {

struct Endpoint {
  std::string host;
  std::uint16_t port = 0;
};

/// Parses "host:port" (IPv4 literal or "localhost"). nullopt on malformed input.
[[nodiscard]] std::optional<Endpoint> parse_endpoint(const std::string& text);

/// Datagram transport. One receive() returns at most one datagram.
class Link {
 public:
  Link() = default;
  Link(const Link&) = delete;
  Link& operator=(const Link&) = delete;
  Link(Link&&) = delete;
  Link& operator=(Link&&) = delete;
  virtual ~Link() = default;

  /// Sends one datagram; false when there is nowhere to send it.
  [[nodiscard]] virtual bool send(const Bytes& datagram) = 0;
  /// Waits up to `timeout` for a datagram; false on timeout.
  [[nodiscard]] virtual bool receive(Bytes& out, std::chrono::milliseconds timeout) = 0;
  [[nodiscard]] virtual std::string describe() const = 0;
  /// True when the link replays a recording and cannot reach a vehicle.
  [[nodiscard]] virtual bool is_replay() const { return false; }
};

/// IPv4 UDP link. Replies go to the peer that last sent a datagram, which is
/// how PX4/ArduPilot SITL expects a ground station to behave.
class UdpLink final : public Link {
 public:
  /// Binds `bind_port` on all interfaces (0 picks a free port). `peer` is the
  /// initial destination before anything has been received.
  explicit UdpLink(std::uint16_t bind_port, const std::optional<Endpoint>& peer = std::nullopt);
  ~UdpLink() override;
  UdpLink(const UdpLink&) = delete;
  UdpLink& operator=(const UdpLink&) = delete;
  UdpLink(UdpLink&&) = delete;
  UdpLink& operator=(UdpLink&&) = delete;

  [[nodiscard]] bool send(const Bytes& datagram) override;
  [[nodiscard]] bool receive(Bytes& out, std::chrono::milliseconds timeout) override;
  [[nodiscard]] std::string describe() const override;
  [[nodiscard]] std::uint16_t local_port() const { return local_port_; }
  [[nodiscard]] bool has_peer() const { return has_peer_; }

 private:
  int fd_ = -1;
  std::uint16_t local_port_ = 0;
  bool has_peer_ = false;
  std::uint32_t peer_addr_ = 0;  // network byte order
  std::uint16_t peer_port_ = 0;  // network byte order
};

/// Replays a recording with its original timing divided by `speed`.
class ReplayLink final : public Link {
 public:
  ReplayLink(std::vector<TlogRecord> records, double speed, std::string source);

  [[nodiscard]] bool send(const Bytes& /*datagram*/) override { return false; }
  [[nodiscard]] bool receive(Bytes& out, std::chrono::milliseconds timeout) override;
  [[nodiscard]] std::string describe() const override;
  [[nodiscard]] bool is_replay() const override { return true; }
  [[nodiscard]] bool finished() const { return next_ >= records_.size(); }

 private:
  [[nodiscard]] std::chrono::steady_clock::time_point due(
      std::size_t index, std::chrono::steady_clock::time_point start) const;

  std::vector<TlogRecord> records_;
  double speed_;
  std::string source_;
  std::size_t next_ = 0;
  std::optional<std::chrono::steady_clock::time_point> start_;
};

}  // namespace mavgs
