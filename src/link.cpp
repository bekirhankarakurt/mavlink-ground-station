#include "mavgs/link.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <thread>

namespace mavgs {

std::optional<Endpoint> parse_endpoint(const std::string& text) {
  const auto colon = text.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 >= text.size()) return std::nullopt;
  Endpoint ep;
  ep.host = text.substr(0, colon);
  if (ep.host == "localhost") ep.host = "127.0.0.1";
  in_addr probe{};
  if (inet_pton(AF_INET, ep.host.c_str(), &probe) != 1) return std::nullopt;
  const std::string port_text = text.substr(colon + 1);
  char* end = nullptr;
  const long port = std::strtol(port_text.c_str(), &end, 10);
  if (*end != '\0' || port < 1 || port > 65535) return std::nullopt;
  ep.port = static_cast<std::uint16_t>(port);
  return ep;
}

UdpLink::UdpLink(std::uint16_t bind_port, const std::optional<Endpoint>& peer)
    : fd_(::socket(AF_INET, SOCK_DGRAM, 0)) {
  if (fd_ < 0) throw std::runtime_error("cannot create UDP socket");

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(bind_port);
  if (::bind(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) != 0) {
    ::close(fd_);
    throw std::runtime_error("cannot bind UDP port " + std::to_string(bind_port) +
                             " (already in use?)");
  }
  socklen_t len = sizeof addr;
  (void)::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
  local_port_ = ntohs(addr.sin_port);

  if (peer) {
    in_addr in{};
    (void)inet_pton(AF_INET, peer->host.c_str(), &in);
    peer_addr_ = in.s_addr;
    peer_port_ = htons(peer->port);
    has_peer_ = true;
  }
}

UdpLink::~UdpLink() {
  if (fd_ >= 0) ::close(fd_);
}

bool UdpLink::send(const Bytes& datagram) {
  if (!has_peer_) return false;
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_addr.s_addr = peer_addr_;
  to.sin_port = peer_port_;
  const auto sent = ::sendto(fd_, datagram.data(), datagram.size(), 0,
                             reinterpret_cast<const sockaddr*>(&to), sizeof to);
  return sent == static_cast<ssize_t>(datagram.size());
}

bool UdpLink::receive(Bytes& out, std::chrono::milliseconds timeout) {
  pollfd pfd{fd_, POLLIN, 0};
  if (::poll(&pfd, 1, static_cast<int>(timeout.count())) <= 0) return false;

  std::uint8_t buffer[2048];
  sockaddr_in from{};
  socklen_t from_len = sizeof from;
  const auto n = ::recvfrom(fd_, buffer, sizeof buffer, 0, reinterpret_cast<sockaddr*>(&from),
                            &from_len);
  if (n <= 0) return false;
  peer_addr_ = from.sin_addr.s_addr;
  peer_port_ = from.sin_port;
  has_peer_ = true;
  out.assign(buffer, buffer + n);
  return true;
}

std::string UdpLink::describe() const { return "udp:" + std::to_string(local_port_); }

ReplayLink::ReplayLink(std::vector<TlogRecord> records, double speed, std::string source)
    : records_(std::move(records)), speed_(speed > 0.0 ? speed : 1.0), source_(std::move(source)) {}

std::chrono::steady_clock::time_point ReplayLink::due(std::size_t index,
                                                       std::chrono::steady_clock::time_point start) const {
  const auto first = records_.front().timestamp_us;
  const auto ts = records_[index].timestamp_us;
  const double offset_us = static_cast<double>(ts > first ? ts - first : 0) / speed_;
  return start + std::chrono::microseconds(static_cast<std::int64_t>(offset_us));
}

bool ReplayLink::receive(Bytes& out, std::chrono::milliseconds timeout) {
  if (finished()) {
    std::this_thread::sleep_for(timeout);
    return false;
  }
  if (!start_) start_ = std::chrono::steady_clock::now();

  const auto now = std::chrono::steady_clock::now();
  const auto when = due(next_, *start_);
  if (when > now) {
    const auto wait = std::min<std::chrono::steady_clock::duration>(when - now, timeout);
    std::this_thread::sleep_for(wait);
    if (std::chrono::steady_clock::now() < when) return false;
  }
  out = records_[next_++].frame;
  return true;
}

std::string ReplayLink::describe() const {
  return "replay:" + source_ + " x" + std::to_string(speed_).substr(0, 4);
}

}  // namespace mavgs
