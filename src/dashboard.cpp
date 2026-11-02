#include "mavgs/dashboard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace mavgs {

namespace {

template <typename... Args>
std::string fmt(const char* format, Args... args) {
  char buffer[256];
  std::snprintf(buffer, sizeof buffer, format, args...);
  return buffer;
}

double tidy(double v) { return (std::round(v * 10.0) / 10.0) + 0.0; }  // avoids "-0.0"

const char* kGreen = "\x1b[32m";
const char* kYellow = "\x1b[33m";
const char* kRed = "\x1b[31m";
const char* kBold = "\x1b[1m";
const char* kReset = "\x1b[0m";

std::string paint(bool color, const char* code, const std::string& text) {
  return color ? std::string(code) + text + kReset : text;
}

std::string gps_text(const VehicleState& v) {
  if (!v.gps) return "no data";
  const char* fix = "no fix";
  switch (v.gps->fix_type) {
    case 2: fix = "2D"; break;
    case 3: fix = "3D"; break;
    case 4: fix = "DGPS"; break;
    case 5: fix = "RTK float"; break;
    case 6: fix = "RTK fixed"; break;
    default: break;
  }
  return fmt("%s  sats %d  hdop %.2f", fix, v.gps->satellites, v.gps->hdop);
}

std::string link_state(const Station& s, bool color) {
  if (s.link().is_replay()) return paint(color, kYellow, "REPLAY");
  if (!s.vehicle().seen) return paint(color, kYellow, "WAITING FOR HEARTBEAT");
  if (!s.vehicle_online()) return paint(color, kRed, "LINK LOST");
  return paint(color, kGreen, "LINK OK");
}

std::string mission_text(const Station& s) {
  std::string text;
  switch (s.uploader().state()) {
    case MissionUploader::State::Uploading:
      text = fmt("uploading %zu/%zu", s.uploader().sent(), s.uploader().total());
      break;
    case MissionUploader::State::Failed: text = "upload failed: " + s.uploader().error(); break;
    default:
      text = s.mission_size() > 0 ? fmt("%zu items on vehicle", s.mission_size()) : "none uploaded";
      break;
  }
  if (const auto current = s.vehicle().mission_current) text += fmt("   current wp %u", *current);
  return text;
}

}  // namespace

std::string render_dashboard(const Station& s, const RenderOptions& o) {
  const VehicleState& v = s.vehicle();
  const LinkStats& st = s.stats();
  std::ostringstream out;

  out << paint(o.color, kBold, "MAVLink GCS") << "  " << s.link().describe() << "  system "
      << (v.seen ? std::to_string(v.sysid) : "-") << "  " << link_state(s, o.color) << "\n";
  out << std::string(72, '-') << "\n";

  const std::string armed = v.armed() ? paint(o.color, kRed, "ARMED") : paint(o.color, kGreen, "disarmed");
  out << fmt(" MODE   %-14s", v.mode_name().c_str()) << armed << "   state " << v.status_name() << "\n";

  if (v.position) {
    const auto& p = *v.position;
    out << fmt(" ALT    %7.1f m rel   %7.1f m MSL     HDG %03.0f\n", p.alt_rel, p.alt_msl, p.hdg_deg);
    out << fmt(" POS    %11.7f  %11.7f\n", p.lat, p.lon);
  } else {
    out << " ALT    -\n POS    -\n";
  }
  if (v.hud) {
    out << fmt(" SPEED  gs %5.1f m/s   climb %+5.1f m/s   throttle %3d %%\n", v.hud->groundspeed,
               tidy(v.hud->climb), v.hud->throttle);
  } else {
    out << " SPEED  -\n";
  }
  if (v.attitude) {
    out << fmt(" ATT    roll %+6.1f   pitch %+6.1f   yaw %+7.1f deg\n", tidy(v.attitude->roll),
               tidy(v.attitude->pitch), tidy(v.attitude->yaw));
  } else {
    out << " ATT    -\n";
  }
  out << " GPS    " << gps_text(v) << "\n";
  if (v.battery) {
    const std::string pct = v.battery->percent < 0 ? "--" : std::to_string(v.battery->percent);
    const char* tone = v.battery->percent >= 0 && v.battery->percent < 20 ? kRed : kGreen;
    out << " BATT   " << paint(o.color, tone, fmt("%.2f V  %.1f A  %s %%", v.battery->volts, v.battery->amps, pct.c_str()))
        << "\n";
  } else {
    out << " BATT   -\n";
  }
  out << " MISSION " << mission_text(s) << "\n";

  out << std::string(72, '-') << "\n";
  out << fmt(" LINK   rx %llu   loss %.1f %% (%llu lost)   %.1f msg/s   %.1f kB/s   crc errors %llu\n",
             static_cast<unsigned long long>(st.received()), st.loss_percent(),
             static_cast<unsigned long long>(st.lost()), st.msg_rate(), st.byte_rate() / 1000.0,
             static_cast<unsigned long long>(st.crc_errors()));
  if (st.has_latency()) {
    out << fmt(" PING   last %.1f ms   avg %.1f ms   min %.1f   max %.1f\n", st.latency_last_ms(),
               st.latency_avg_ms(), st.latency_min_ms(), st.latency_max_ms());
  } else {
    out << " PING   -\n";
  }

  out << std::string(72, '-') << "\n";
  const auto& events = s.events();
  const std::size_t shown = std::min(o.max_events, events.size());
  for (std::size_t i = events.size() - shown; i < events.size(); ++i) {
    out << " " << render_event(events[i], o.origin_us) << "\n";
  }
  for (std::size_t i = shown; i < o.max_events; ++i) out << "\n";

  out << std::string(72, '-') << "\n";
  out << " [a]rm [d]isarm [t]akeoff [l]and [r]tl [m]ission [s]tart [q]uit\n";
  return out.str();
}

std::string render_event(const Event& event, std::uint64_t origin_us) {
  const double t = static_cast<double>(event.t_us - origin_us) / 1e6;
  return fmt("[%7.1fs] %s", t, event.text.c_str());
}

std::string render_status_line(const Station& s, double elapsed_s) {
  const VehicleState& v = s.vehicle();
  const LinkStats& st = s.stats();
  std::string line = fmt("[%7.1fs] %-12s %-8s", elapsed_s, v.mode_name().c_str(),
                         v.armed() ? "ARMED" : "disarmed");
  if (v.position) line += fmt(" alt %6.1f m", v.position->alt_rel);
  if (v.hud) line += fmt(" gs %4.1f m/s", v.hud->groundspeed);
  if (v.battery && v.battery->percent >= 0) {
    line += fmt(" batt %.1f V %d %%", v.battery->volts, v.battery->percent);
  }
  if (v.gps) line += fmt(" gps %d/%d", v.gps->fix_type, v.gps->satellites);
  if (v.mission_current) line += fmt(" wp %u", *v.mission_current);
  line += fmt(" | rx %llu loss %.1f%% %.1f msg/s", static_cast<unsigned long long>(st.received()),
              st.loss_percent(), st.msg_rate());
  if (st.has_latency()) line += fmt(" ping %.1f ms", st.latency_last_ms());
  return line;
}

}  // namespace mavgs
