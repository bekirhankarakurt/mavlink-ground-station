#include <cstdio>
#include <fstream>

#include "check.hpp"
#include "harness.hpp"
#include "mavgs/cli.hpp"
#include "mavgs/script.hpp"

using namespace mavgs;
using testing_support::Harness;

namespace {

ArgParser make_parser() {
  return ArgParser("prog", "summary",
                   {{"port", "N", "port", "1"}, {"quiet", "", "quiet", ""}, {"name", "S", "name", ""}});
}

void parse(ArgParser& p, std::vector<const char*> args) {
  args.insert(args.begin(), "prog");
  p.parse(static_cast<int>(args.size()), args.data());
}

}  // namespace

TEST(cli_parses_values_flags_and_equals_form) {
  auto p = make_parser();
  parse(p, {"--port", "99", "--quiet", "--name=abc"});
  CHECK(p.has("quiet"));
  CHECK_EQ(p.get("name"), std::string("abc"));
  CHECK_NEAR(p.get_number("port", 1, 1, 65535), 99.0, 1e-9);
  CHECK_NEAR(p.get_number("missing", 7, 0, 10), 7.0, 1e-9);
}

TEST(cli_rejects_bad_input) {
  auto a = make_parser();
  CHECK_THROWS(parse(a, {"--nope"}));
  auto b = make_parser();
  CHECK_THROWS(parse(b, {"--port"}));
  auto c = make_parser();
  CHECK_THROWS(parse(c, {"positional"}));
  auto d = make_parser();
  CHECK_THROWS(parse(d, {"--quiet=1"}));
  auto e = make_parser();
  parse(e, {"--port", "abc"});
  CHECK_THROWS(e.get_number("port", 1, 1, 10));
  auto f = make_parser();
  parse(f, {"--port", "99"});
  CHECK_THROWS(f.get_number("port", 1, 1, 10));
}

TEST(cli_usage_lists_options_and_defaults) {
  const std::string usage = make_parser().usage();
  CHECK(usage.find("--port N") != std::string::npos);
  CHECK(usage.find("(default: 1)") != std::string::npos);
  CHECK(usage.find("summary") != std::string::npos);
}

TEST(script_parses_steps) {
  const auto steps = parse_script("arm,takeoff:12.5,wait:3,rtl,wait-disarmed,land,start,disarm");
  CHECK_EQ(steps.size(), std::size_t{8});
  CHECK(steps[1].kind == ScriptStep::Kind::Takeoff);
  CHECK_NEAR(steps[1].value, 12.5, 1e-9);
  CHECK(steps[2].kind == ScriptStep::Kind::Wait);
}

TEST(script_loads_mission_file_at_parse_time) {
  const std::string path = "/tmp/mavgs_test_script_mission.txt";
  std::ofstream(path) << "takeoff 10\nrtl\n";
  const auto steps = parse_script("mission:" + path);
  CHECK_EQ(steps.at(0).items.size(), std::size_t{2});
  std::remove(path.c_str());
  CHECK_THROWS(parse_script("mission:/nonexistent/m.txt"));
}

TEST(script_rejects_bad_steps) {
  CHECK_THROWS(parse_script(""));
  CHECK_THROWS(parse_script("fly"));
  CHECK_THROWS(parse_script("takeoff"));
  CHECK_THROWS(parse_script("takeoff:abc"));
  CHECK_THROWS(parse_script("arm:1"));
  CHECK_THROWS(parse_script("wait:-1"));
  CHECK_THROWS(parse_script("mission"));
}

TEST(script_runs_a_full_flight_against_the_simulator) {
  SimConfig cfg;
  cfg.speed = 10.0;
  Harness h(cfg);
  ScriptRunner script(parse_script("arm,takeoff:10,wait:6,rtl,wait-disarmed"));
  while (!script.finished() && h.now_us < 200'000'000) {
    h.run(0.02);
    script.advance(h.station);
  }
  CHECK(script.finished());
  CHECK(!script.failed());
  CHECK(!h.sim.armed());
}

TEST(script_fails_when_vehicle_rejects_a_command) {
  Harness h;
  ScriptRunner script(parse_script("takeoff:10"));  // not armed -> denied
  h.run(2);
  while (!script.finished() && h.now_us < 20'000'000) {
    h.run(0.02);
    script.advance(h.station);
  }
  CHECK(script.failed());
  CHECK(script.error().find("takeoff:10") != std::string::npos);
}

TEST(script_waits_for_link_before_first_step) {
  testing_support::LoopLink link;
  Station station(link, {});
  ScriptRunner script(parse_script("arm"));
  station.service(1, std::chrono::milliseconds(0));
  script.advance(station);
  CHECK(!script.finished());
  CHECK(link.sent.size() == 1);  // only the GCS heartbeat, no command
}
