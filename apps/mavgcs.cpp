// mavgcs - terminal MAVLink v2 ground control station.

#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>

#include "mavgs/cli.hpp"
#include "mavgs/clock.hpp"
#include "mavgs/dashboard.hpp"
#include "mavgs/link.hpp"
#include "mavgs/script.hpp"
#include "mavgs/station.hpp"
#include "mavgs/terminal.hpp"
#include "mavgs/tlog.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int /*signal*/) { g_stop = 1; }

enum class Ui : std::uint8_t { Dashboard, Log };

constexpr std::chrono::milliseconds kPollInterval{20};
constexpr std::uint64_t kDashboardPeriodUs = 200'000;
constexpr std::uint64_t kStatusPeriodUs = 1'000'000;

std::vector<mavgs::OptionSpec> option_specs() {
  return {
      {"port", "N", "UDP port to listen on", "14550"},
      {"connect", "HOST:PORT", "send to this address until the vehicle has been heard", ""},
      {"replay", "FILE", "replay a .tlog recording instead of opening a link", ""},
      {"speed", "X", "replay speed multiplier", "1"},
      {"record", "FILE", "record received telemetry to a .tlog file", ""},
      {"mission", "FILE", "waypoint file for the [m] key", ""},
      {"takeoff-alt", "M", "altitude for the [t] key, metres above ground", "10"},
      {"script", "STEPS", "run steps, e.g. arm,takeoff:10,wait:20,rtl,wait-disarmed", ""},
      {"ui", "dashboard|log", "output style", "dashboard on a terminal, else log"},
      {"duration", "S", "exit after this many seconds", "run until quit"},
      {"sysid", "N", "system id used by this station", "255"},
      {"vehicle", "N", "track this vehicle system id", "first autopilot heard"},
      {"no-color", "", "disable ANSI colors", ""},
      {"help", "", "show this help", ""},
  };
}

struct Options {
  std::uint16_t port = 14550;
  std::optional<mavgs::Endpoint> connect;
  std::string replay;
  double speed = 1.0;
  std::string record;
  std::vector<mavgs::MissionItem> mission;
  double takeoff_alt = 10.0;
  std::vector<mavgs::ScriptStep> script;
  Ui ui = Ui::Log;
  double duration = 0.0;
  mavgs::StationConfig station;
  bool color = false;
};

Options read_options(const mavgs::ArgParser& args) {
  Options o;
  o.port = static_cast<std::uint16_t>(args.get_number("port", 14550, 1, 65535));
  if (args.has("connect")) {
    o.connect = mavgs::parse_endpoint(args.get("connect"));
    if (!o.connect) throw std::invalid_argument("--connect expects HOST:PORT (IPv4)");
  }
  o.replay = args.get("replay");
  o.speed = args.get_number("speed", 1.0, 0.01, 1000);
  o.record = args.get("record");
  if (args.has("mission")) o.mission = mavgs::load_mission(args.get("mission"));
  o.takeoff_alt = args.get_number("takeoff-alt", 10.0, 1, 10000);
  if (args.has("script")) o.script = mavgs::parse_script(args.get("script"));
  o.duration = args.get_number("duration", 0.0, 0, 86400 * 7);
  o.station.sysid = static_cast<std::uint8_t>(args.get_number("sysid", 255, 1, 255));
  if (args.has("vehicle")) {
    o.station.vehicle_sysid = static_cast<std::uint8_t>(args.get_number("vehicle", 1, 1, 255));
  }
  if (!o.replay.empty() && !o.record.empty()) {
    throw std::invalid_argument("--record cannot be combined with --replay");
  }

  const bool tty = isatty(STDOUT_FILENO) != 0;
  const std::string ui = args.get("ui", tty ? "dashboard" : "log");
  if (ui == "dashboard") {
    o.ui = Ui::Dashboard;
  } else if (ui == "log") {
    o.ui = Ui::Log;
  } else {
    throw std::invalid_argument("--ui must be 'dashboard' or 'log'");
  }
  o.color = tty && !args.has("no-color") && std::getenv("NO_COLOR") == nullptr;
  return o;
}

void handle_key(int key, mavgs::Station& station, const Options& o, bool& quit) {
  switch (key) {
    case 'a': (void)station.arm(); break;
    case 'd': (void)station.disarm(); break;
    case 't': (void)station.takeoff(o.takeoff_alt); break;
    case 'l': (void)station.land(); break;
    case 'r': (void)station.return_to_launch(); break;
    case 's': (void)station.start_mission(); break;
    case 'm':
      if (o.mission.empty()) {
        station.note("no mission loaded: start with --mission FILE");
      } else {
        (void)station.upload_mission(o.mission);
      }
      break;
    case 'q': quit = true; break;
    default: break;
  }
}

int run(const Options& o) {
  std::unique_ptr<mavgs::Link> link;
  if (!o.replay.empty()) {
    link = std::make_unique<mavgs::ReplayLink>(mavgs::read_tlog(o.replay), o.speed, o.replay);
  } else {
    link = std::make_unique<mavgs::UdpLink>(o.port, o.connect);
  }
  std::unique_ptr<mavgs::TlogWriter> recorder;
  if (!o.record.empty()) recorder = std::make_unique<mavgs::TlogWriter>(o.record);

  mavgs::Station station(*link, o.station, recorder.get());
  mavgs::ScriptRunner script(o.script);
  const bool has_script = !o.script.empty();

  mavgs::KeyboardMode keyboard;
  const bool dashboard = o.ui == Ui::Dashboard;
  if (dashboard) std::fputs("\x1b[2J\x1b[?25l", stdout);

  const std::uint64_t origin = mavgs::monotonic_us();
  std::uint64_t last_draw = 0;
  std::uint64_t next_event = 0;
  bool quit = false;
  int exit_code = 0;
  std::uint64_t replay_done_at = 0;

  if (!dashboard) std::printf("listening: %s\n", link->describe().c_str());

  while (g_stop == 0 && !quit) {
    const std::uint64_t now = mavgs::monotonic_us();
    station.service(now, kPollInterval);
    handle_key(keyboard.poll_key(), station, o, quit);

    if (has_script) {
      script.advance(station);
      if (script.failed()) {
        station.note(script.error());
        exit_code = 1;
        quit = true;
      } else if (script.finished()) {
        quit = true;
      }
    }

    if (dashboard) {
      if (now - last_draw >= kDashboardPeriodUs || quit) {
        last_draw = now;
        mavgs::RenderOptions ro;
        ro.color = o.color;
        ro.origin_us = origin;
        std::string frame = mavgs::render_dashboard(station, ro);
        std::string out = "\x1b[H";
        for (const char c : frame) {
          if (c == '\n') out += "\x1b[K";
          out.push_back(c);
        }
        out += "\x1b[J";
        std::fputs(out.c_str(), stdout);
        std::fflush(stdout);
      }
    } else {
      for (const auto& e : station.events()) {
        if (e.id >= next_event) std::printf("%s\n", mavgs::render_event(e, origin).c_str());
      }
      next_event = station.event_count();
      if (now - last_draw >= kStatusPeriodUs) {
        last_draw = now;
        std::printf("%s\n", mavgs::render_status_line(station, static_cast<double>(now - origin) / 1e6).c_str());
      }
      std::fflush(stdout);
    }

    if (o.duration > 0 && static_cast<double>(now - origin) / 1e6 >= o.duration) {
      if (has_script && !script.finished()) {
        station.note("script did not finish within --duration");
        exit_code = 1;
      }
      quit = true;
    }
    const auto* replay = dynamic_cast<const mavgs::ReplayLink*>(link.get());
    if (replay != nullptr && replay->finished() && !dashboard) {
      if (replay_done_at == 0) replay_done_at = now;
      if (now - replay_done_at > 500'000) quit = true;
    }
  }

  if (dashboard) std::fputs("\x1b[?25h\n", stdout);
  if (!dashboard) {
    for (const auto& e : station.events()) {
      if (e.id >= next_event) std::printf("%s\n", mavgs::render_event(e, origin).c_str());
    }
  }
  const auto& st = station.stats();
  std::printf("summary: %llu frames, %.1f%% loss, %llu crc errors\n",
              static_cast<unsigned long long>(st.received()), st.loss_percent(),
              static_cast<unsigned long long>(st.crc_errors()));
  if (recorder) {
    std::printf("recorded %llu frames to %s\n",
                static_cast<unsigned long long>(recorder->records()), o.record.c_str());
  }
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  mavgs::ArgParser args("mavgcs", "mavgcs - terminal MAVLink v2 ground control station", option_specs());
  try {
    args.parse(argc, argv);
    if (args.has("help")) {
      std::cout << args.usage();
      return 0;
    }
    const Options options = read_options(args);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    return run(options);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "mavgcs: %s\n", e.what());
    return 2;
  }
}
