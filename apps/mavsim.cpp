// mavsim - MAVLink v2 vehicle simulator for testing mavgcs without hardware.

#include <csignal>
#include <cstdio>
#include <exception>
#include <iostream>
#include <sstream>

#include "mavgs/cli.hpp"
#include "mavgs/clock.hpp"
#include "mavgs/link.hpp"
#include "mavgs/sim.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int /*signal*/) { g_stop = 1; }

std::vector<mavgs::OptionSpec> option_specs() {
  return {
      {"target", "HOST:PORT", "ground station address to stream telemetry to", "127.0.0.1:14550"},
      {"port", "N", "local UDP port", "14580"},
      {"speed", "X", "simulation speed multiplier", "1"},
      {"loss", "P", "probability (0..1) of dropping each outgoing frame", "0"},
      {"seed", "N", "random seed for the loss model", "1"},
      {"sysid", "N", "MAVLink system id", "1"},
      {"home", "LAT,LON,ALT", "home position in degrees and metres AMSL", "47.397742,8.545594,488"},
      {"duration", "S", "exit after this many seconds", "run until interrupted"},
      {"help", "", "show this help", ""},
  };
}

void parse_home(const std::string& text, mavgs::SimConfig& config) {
  std::istringstream in(text);
  char c1 = 0;
  char c2 = 0;
  double lat = 0;
  double lon = 0;
  double alt = 0;
  if (!(in >> lat >> c1 >> lon >> c2 >> alt) || c1 != ',' || c2 != ',' || lat < -90 || lat > 90 ||
      lon < -180 || lon > 180) {
    throw std::invalid_argument("--home expects LAT,LON,ALT");
  }
  config.home_lat = lat;
  config.home_lon = lon;
  config.home_alt_msl = alt;
}

}  // namespace

int main(int argc, char** argv) {
  mavgs::ArgParser args("mavsim", "mavsim - MAVLink v2 vehicle simulator", option_specs());
  try {
    args.parse(argc, argv);
    if (args.has("help")) {
      std::cout << args.usage();
      return 0;
    }
    mavgs::SimConfig config;
    config.speed = args.get_number("speed", 1.0, 0.1, 100);
    config.loss = args.get_number("loss", 0.0, 0.0, 1.0);
    config.seed = static_cast<std::uint32_t>(args.get_number("seed", 1, 1, 4294967295.0));
    config.sysid = static_cast<std::uint8_t>(args.get_number("sysid", 1, 1, 255));
    if (args.has("home")) parse_home(args.get("home"), config);
    const double duration = args.get_number("duration", 0.0, 0, 86400 * 7);

    const auto target = mavgs::parse_endpoint(args.get("target", "127.0.0.1:14550"));
    if (!target) throw std::invalid_argument("--target expects HOST:PORT (IPv4)");
    mavgs::UdpLink link(static_cast<std::uint16_t>(args.get_number("port", 14580, 0, 65535)), target);
    mavgs::VehicleSim sim(config);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::printf("mavsim: system %d, udp %u -> %s:%u, speed x%.1f, loss %.0f%%\n", config.sysid,
                link.local_port(), target->host.c_str(), target->port, config.speed, config.loss * 100);
    std::fflush(stdout);

    const std::uint64_t origin = mavgs::monotonic_us();
    mavgs::Bytes datagram;
    while (g_stop == 0) {
      const std::uint64_t now = mavgs::monotonic_us();
      if (duration > 0 && static_cast<double>(now - origin) / 1e6 >= duration) break;

      if (link.receive(datagram, std::chrono::milliseconds(5))) {
        for (const auto& reply : sim.handle(datagram)) (void)link.send(reply);
      }
      for (const auto& frame : sim.step(now)) (void)link.send(frame);
      for (const auto& line : sim.take_log()) {
        std::printf("[%7.1fs] %s\n", static_cast<double>(now - origin) / 1e6, line.c_str());
      }
      std::fflush(stdout);
    }
    std::printf("mavsim: stopped (%llu frames dropped by loss model)\n",
                static_cast<unsigned long long>(sim.frames_dropped()));
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "mavsim: %s\n", e.what());
    return 2;
  }
}
