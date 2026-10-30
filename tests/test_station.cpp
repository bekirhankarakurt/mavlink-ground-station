#include "check.hpp"
#include "harness.hpp"
#include "mavgs/dashboard.hpp"
#include "mavgs/enums.hpp"

using namespace mavgs;
using testing_support::Harness;

namespace {

bool has_event(const Station& s, const std::string& needle) {
  for (const auto& e : s.events()) {
    if (e.text.find(needle) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

TEST(station_detects_vehicle_and_tracks_telemetry) {
  Harness h;
  CHECK(!h.station.vehicle_online());
  h.run(3);
  CHECK(h.station.vehicle_online());
  CHECK_EQ(h.station.vehicle().sysid, std::uint8_t{1});
  CHECK(h.station.vehicle().position.has_value());
  CHECK(h.station.vehicle().battery.has_value());
  CHECK(h.station.vehicle().attitude.has_value());
  CHECK(h.station.vehicle().gps.has_value());
  CHECK(h.station.vehicle().hud.has_value());
  CHECK_EQ(h.station.vehicle().mode_name(), std::string("POSCTL"));
  CHECK_NEAR(h.station.vehicle().position->lat, 47.397742, 1e-6);
  CHECK(has_event(h.station, "vehicle detected"));
}

TEST(station_measures_zero_loss_and_latency) {
  Harness h;
  h.run(6);
  CHECK_EQ(h.station.stats().lost(), std::uint64_t{0});
  CHECK(h.station.stats().has_latency());
  CHECK(h.station.stats().latency_avg_ms() < 100.0);
  CHECK(h.station.stats().msg_rate() > 10.0);
}

TEST(station_reports_loss_from_lossy_link) {
  SimConfig cfg;
  cfg.loss = 0.25;
  cfg.seed = 3;
  Harness h(cfg);
  h.run(30);
  CHECK(h.station.stats().loss_percent() > 15.0);
  CHECK(h.station.stats().loss_percent() < 35.0);
}

TEST(station_refuses_commands_without_vehicle) {
  testing_support::LoopLink link;
  Station station(link, {});
  station.service(1, std::chrono::milliseconds(0));
  CHECK(!station.arm());
  CHECK(!station.upload_mission({MissionItem{}}));
  CHECK(has_event(station, "no vehicle"));
  CHECK(link.sent.size() == 1);  // GCS heartbeat only
}

TEST(station_takeoff_needs_position) {
  testing_support::LoopLink link;
  Station station(link, {});
  Packer vehicle(1, 1);
  Heartbeat hb;
  hb.type = 2;
  hb.autopilot = 12;
  link.inbox.push_back(vehicle.pack(hb));
  station.service(1'000'000, std::chrono::milliseconds(0));
  CHECK(!station.takeoff(10));
  CHECK(has_event(station, "no position"));
}

TEST(station_command_flow_and_events) {
  Harness h;
  h.run(2);
  CHECK(h.station.arm());
  CHECK(h.station.command_status() == CommandStatus::Pending);
  CHECK(!h.station.disarm());  // busy
  h.run(0.2);
  CHECK(h.station.command_status() == CommandStatus::Accepted);
  CHECK(has_event(h.station, "ARM/DISARM: ACCEPTED"));
  h.run(1.5);
  CHECK(h.station.vehicle().armed());
  CHECK(has_event(h.station, "vehicle armed"));
}

TEST(station_retries_command_then_times_out) {
  testing_support::LoopLink link;
  Station station(link, {});
  Packer vehicle(1, 1);
  Heartbeat hb;
  hb.type = static_cast<std::uint8_t>(MavType::Quadrotor);
  hb.autopilot = static_cast<std::uint8_t>(MavAutopilot::Px4);
  link.inbox.push_back(vehicle.pack(hb));
  station.service(1'000'000, std::chrono::milliseconds(0));
  CHECK(station.arm());
  link.sent.clear();
  std::uint64_t t = 1'000'000;
  int command_frames = 0;
  while (t < 10'000'000) {
    t += 100'000;
    station.service(t, std::chrono::milliseconds(0));
    Parser p;
    for (const auto& d : link.sent) {
      for (const auto& f : p.feed(d)) command_frames += f.frame.msgid == CommandLong::kId ? 1 : 0;
    }
    link.sent.clear();
    link.inbox.push_back(vehicle.pack(hb));  // keep the link alive
  }
  CHECK_EQ(command_frames, Station::kCommandAttempts - 1);  // initial send was cleared above
  CHECK(station.command_status() == CommandStatus::TimedOut);
  CHECK(has_event(station, "no response"));
}

TEST(station_reports_link_lost_and_restored) {
  Harness h;
  h.run(3);
  CHECK(h.station.vehicle_online());
  h.now_us += 5'000'000;
  h.station.service(h.now_us, std::chrono::milliseconds(0));
  CHECK(!h.station.vehicle_online());
  CHECK(has_event(h.station, "link lost"));
  h.run(2);
  CHECK(h.station.vehicle_online());
  CHECK(has_event(h.station, "link restored"));
}

TEST(station_ignores_other_systems_once_tracking) {
  testing_support::LoopLink link;
  StationConfig cfg;
  cfg.vehicle_sysid = 2;
  Station station(link, cfg);
  Heartbeat hb;
  hb.type = 2;
  hb.autopilot = 12;
  Packer other(1, 1);
  Packer wanted(2, 1);
  link.inbox.push_back(other.pack(hb));
  link.inbox.push_back(wanted.pack(hb));
  station.service(1'000'000, std::chrono::milliseconds(0));
  CHECK_EQ(station.vehicle().sysid, std::uint8_t{2});
}

TEST(station_counts_crc_errors) {
  testing_support::LoopLink link;
  Station station(link, {});
  Packer vehicle(1, 1);
  Bytes bad = vehicle.pack(Heartbeat{});
  bad[11] = static_cast<std::uint8_t>(bad[11] ^ 0xFF);
  link.inbox.push_back(bad);
  station.service(1, std::chrono::milliseconds(0));
  CHECK_EQ(station.stats().crc_errors(), std::uint64_t{1});
}

TEST(station_mission_upload_failure_is_reported) {
  testing_support::LoopLink link;
  Station station(link, {});
  Packer vehicle(1, 1);
  Heartbeat hb;
  hb.type = 2;
  hb.autopilot = 12;
  link.inbox.push_back(vehicle.pack(hb));
  station.service(1'000'000, std::chrono::milliseconds(0));
  CHECK(station.upload_mission({MissionItem{}}));
  link.inbox.push_back(vehicle.pack(MissionAck{255, 190, 1}));
  station.service(1'100'000, std::chrono::milliseconds(0));
  CHECK(has_event(station, "mission upload failed"));
}

TEST(station_recorded_stream_replays_identically) {
  const std::string path = "/tmp/mavgs_test_station.tlog";
  {
    Harness h;
    TlogWriter writer(path);
    Station recorded(h.link, {}, &writer);
    for (int i = 0; i < 150; ++i) {
      h.now_us += 20'000;
      recorded.service(h.now_us, std::chrono::milliseconds(0));
      h.link.sent.clear();
      for (auto& frame : h.sim.step(h.now_us)) h.link.inbox.push_back(std::move(frame));
    }
    CHECK(writer.records() > 50);
  }
  const auto records = read_tlog(path);
  ReplayLink replay(records, 1000.0, path);
  Station viewer(replay, {});
  std::uint64_t t = 0;
  for (int i = 0; i < 200; ++i) {
    t += 20'000;
    viewer.service(t, std::chrono::milliseconds(1));
  }
  CHECK(replay.finished());
  CHECK_EQ(viewer.stats().received(), static_cast<std::uint64_t>(records.size()));
  CHECK(viewer.vehicle().position.has_value());
  CHECK(!viewer.arm());
  CHECK(has_event(viewer, "replay mode"));
  std::remove(path.c_str());
}

TEST(dashboard_renders_live_values) {
  Harness h;
  h.run(4);
  RenderOptions opts;
  const std::string text = render_dashboard(h.station, opts);
  CHECK(text.find("MAVLink GCS") != std::string::npos);
  CHECK(text.find("LINK OK") != std::string::npos);
  CHECK(text.find("POSCTL") != std::string::npos);
  CHECK(text.find("47.3977420") != std::string::npos);
  CHECK(text.find("sats 12") != std::string::npos);
  CHECK(text.find("loss 0.0 %") != std::string::npos);
  CHECK(text.find("\x1b[") == std::string::npos);  // no escape codes without color
  opts.color = true;
  CHECK(render_dashboard(h.station, opts).find("\x1b[") != std::string::npos);
}

TEST(dashboard_handles_empty_station) {
  testing_support::LoopLink link;
  Station station(link, {});
  const std::string text = render_dashboard(station, {});
  CHECK(text.find("WAITING FOR HEARTBEAT") != std::string::npos);
  CHECK(render_status_line(station, 1.5).find("rx 0") != std::string::npos);
}

TEST(vehicle_state_decodes_mode_names) {
  VehicleState v;
  Heartbeat hb;
  hb.autopilot = 12;
  hb.base_mode = 0x81;
  hb.custom_mode = px4::custom_mode(px4::kMainAuto, px4::kSubRtl);
  v.apply(hb, 1);
  CHECK(v.armed());
  CHECK_EQ(v.mode_name(), std::string("AUTO.RTL"));
  hb.autopilot = 3;
  hb.custom_mode = 6;
  v.apply(hb, 2);
  CHECK_EQ(v.mode_name(), std::string("RTL"));
}
