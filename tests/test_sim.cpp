#include <map>
#include <sstream>

#include "check.hpp"
#include "harness.hpp"
#include "mavgs/enums.hpp"

using namespace mavgs;
using testing_support::Harness;

namespace {

SimConfig fast() {
  SimConfig c;
  c.speed = 10.0;
  return c;
}

Bytes command(std::uint16_t cmd, float p1, float p7 = 0) {
  Packer packer(255, 190);
  CommandLong c;
  c.command = cmd;
  c.param1 = p1;
  c.param7 = p7;
  c.target_system = 1;
  c.target_component = 1;
  return packer.pack(c);
}

std::uint8_t ack_result(const std::vector<Bytes>& replies) {
  Parser parser;
  for (const auto& r : replies) {
    for (const auto& f : parser.feed(r)) {
      if (const auto m = decode(f.frame)) {
        if (const auto* ack = std::get_if<CommandAck>(&*m)) return ack->result;
      }
    }
  }
  return 255;
}

}  // namespace

TEST(sim_streams_all_telemetry_types) {
  VehicleSim sim{SimConfig{}};
  Parser parser;
  std::map<std::uint32_t, int> seen;
  for (std::uint64_t t = 0; t <= 2'000'000; t += 20'000) {
    for (const auto& wire : sim.step(t)) {
      for (const auto& f : parser.feed(wire)) ++seen[f.frame.msgid];
    }
  }
  CHECK_EQ(parser.stats().crc_errors, std::uint64_t{0});
  CHECK(seen[0] >= 2);    // HEARTBEAT ~1 Hz
  CHECK(seen[1] >= 2);    // SYS_STATUS
  CHECK(seen[30] >= 18);  // ATTITUDE ~10 Hz
  CHECK(seen[33] >= 9);   // GLOBAL_POSITION_INT ~5 Hz
  CHECK(seen[24] >= 3);   // GPS_RAW_INT ~2 Hz
  CHECK(seen[74] >= 9);   // VFR_HUD ~5 Hz
}

TEST(sim_arm_takeoff_requires_arming) {
  VehicleSim sim{SimConfig{}};
  CHECK_EQ(ack_result(sim.handle(command(22, 0, 500))), static_cast<std::uint8_t>(MavResult::Denied));
  CHECK_EQ(ack_result(sim.handle(command(400, 1))), static_cast<std::uint8_t>(MavResult::Accepted));
  CHECK(sim.armed());
  CHECK_EQ(ack_result(sim.handle(command(9999, 0))), static_cast<std::uint8_t>(MavResult::Unsupported));
}

TEST(sim_takeoff_reaches_altitude_then_holds) {
  Harness h(fast());
  h.run(2);
  CHECK(h.station.arm());
  h.run(1);
  CHECK(h.station.vehicle().armed());
  CHECK(h.station.takeoff(20.0));
  h.run(10);
  CHECK(h.sim.mode() == VehicleSim::Mode::Hold);
  CHECK_NEAR(h.sim.altitude(), 20.0, 0.6);
  CHECK_NEAR(h.station.vehicle().position->alt_rel, 20.0, 1.0);
}

TEST(sim_cannot_disarm_in_flight) {
  Harness h(fast());
  h.run(2);
  CHECK(h.station.arm());
  h.run(0.5);
  CHECK(h.station.takeoff(10.0));
  h.run(6);
  CHECK(h.station.disarm());
  h.run(1);
  CHECK(h.station.command_status() == CommandStatus::Rejected);
  CHECK(h.sim.armed());
}

TEST(sim_rtl_returns_home_lands_and_disarms) {
  SimConfig cfg = fast();
  Harness h(cfg);
  h.run(2);
  CHECK(h.station.arm());
  h.run(0.5);
  CHECK(h.station.takeoff(20.0));
  h.run(8);
  const auto mission = std::vector<MissionItem>{
      {16, 6, 0, 0, 0, 0, 473987420, 85455940, 20}};  // 111 m north of home
  CHECK(h.station.upload_mission(mission));
  CHECK(h.run_until([&] { return h.station.uploader().state() == MissionUploader::State::Done; }, 5));
  CHECK(h.station.start_mission());
  h.run(25);
  CHECK(h.sim.north() > 100.0);
  CHECK(h.station.return_to_launch());
  CHECK(h.run_until([&] { return !h.sim.armed(); }, 60));
  CHECK_NEAR(h.sim.north(), 0.0, 2.0);
  CHECK_NEAR(h.sim.altitude(), 0.0, 0.1);
  CHECK(h.run_until([&] { return !h.station.vehicle().armed(); }, 3));
}

TEST(sim_land_command_descends_and_disarms) {
  Harness h(fast());
  h.run(2);
  CHECK(h.station.arm());
  h.run(0.5);
  CHECK(h.station.takeoff(10.0));
  h.run(6);
  CHECK(h.station.land());
  CHECK(h.run_until([&] { return !h.sim.armed(); }, 30));
}

TEST(sim_flies_uploaded_mission_end_to_end) {
  Harness h(fast());
  h.run(2);
  std::istringstream text(
      "takeoff 15\nwaypoint 47.3980 8.5460 25 1\nwaypoint 47.3980 8.5450 25\nland 47.3978 8.5456\n");
  CHECK(h.station.upload_mission(parse_mission(text)));
  CHECK(h.run_until([&] { return h.station.uploader().state() == MissionUploader::State::Done; }, 5));
  CHECK_EQ(h.sim.mission_size(), std::size_t{4});
  CHECK(h.station.arm());
  h.run(0.5);
  CHECK(h.station.start_mission());
  h.run(1);
  CHECK(h.sim.mode() == VehicleSim::Mode::Mission);
  CHECK(h.station.vehicle().mission_current.has_value());
  CHECK(h.run_until([&] { return !h.sim.armed(); }, 120));
  CHECK_NEAR(h.sim.altitude(), 0.0, 0.1);
}

TEST(sim_rejects_mission_start_without_mission_or_arming) {
  VehicleSim sim{SimConfig{}};
  CHECK_EQ(ack_result(sim.handle(command(400, 1))), static_cast<std::uint8_t>(MavResult::Accepted));
  CHECK_EQ(ack_result(sim.handle(command(300, 0))), static_cast<std::uint8_t>(MavResult::Denied));
}

TEST(sim_rejects_empty_mission_count) {
  VehicleSim sim{SimConfig{}};
  Packer packer(255, 190);
  const auto replies = sim.handle(packer.pack(MissionCount{0, 1, 1}));
  Parser parser;
  const auto frames = parser.feed(replies.at(0));
  CHECK_EQ(frames.at(0).frame.msgid, std::uint32_t{47});
}

TEST(sim_loss_model_drops_frames_but_keeps_sequence_gaps) {
  SimConfig cfg;
  cfg.loss = 0.2;
  cfg.seed = 7;
  VehicleSim sim(cfg);
  LinkStats stats;
  Parser parser;
  for (std::uint64_t t = 0; t <= 30'000'000; t += 20'000) {
    for (const auto& wire : sim.step(t)) {
      for (const auto& f : parser.feed(wire)) stats.on_frame(f.frame.sysid, f.frame.compid, f.frame.seq, f.raw.size());
    }
  }
  CHECK(sim.frames_dropped() > 0);
  CHECK(stats.loss_percent() > 15.0 && stats.loss_percent() < 25.0);
}

TEST(sim_is_deterministic_for_a_given_seed) {
  const auto run = [] {
    SimConfig cfg;
    cfg.loss = 0.3;
    cfg.seed = 99;
    VehicleSim sim(cfg);
    std::size_t bytes = 0;
    for (std::uint64_t t = 0; t <= 5'000'000; t += 20'000) {
      for (const auto& w : sim.step(t)) bytes += w.size();
    }
    return bytes;
  };
  CHECK_EQ(run(), run());
}

TEST(sim_answers_ping_requests) {
  VehicleSim sim{SimConfig{}};
  Packer packer(255, 190);
  const auto replies = sim.handle(packer.pack(Ping{123456, 9, 0, 0}));
  Parser parser;
  const auto frames = parser.feed(replies.at(0));
  const auto m = decode(frames.at(0).frame);
  const auto& pong = std::get<Ping>(*m);
  CHECK_EQ(pong.time_usec, std::uint64_t{123456});
  CHECK_EQ(pong.seq, std::uint32_t{9});
  CHECK_EQ(pong.target_system, std::uint8_t{255});
}
