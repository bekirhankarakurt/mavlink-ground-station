#include <sstream>

#include "check.hpp"
#include "mavgs/enums.hpp"
#include "mavgs/mission.hpp"

using namespace mavgs;

namespace {

std::vector<MissionItem> parse(const std::string& text) {
  std::istringstream in(text);
  return parse_mission(in);
}

int error_line(const std::string& text) {
  try {
    (void)parse(text);
  } catch (const MissionParseError& e) {
    return e.line();
  }
  return -1;
}

std::vector<MissionItem> three_items() {
  return parse("takeoff 10\nwaypoint 47.4 8.5 20\nrtl\n");
}

}  // namespace

TEST(parse_valid_mission_with_comments_and_case) {
  const auto items = parse(
      "# header\n\n"
      "TAKEOFF 15   # climb\n"
      "waypoint 47.3977420 8.5455940 20 5\n"
      "land 47.4 8.5\n"
      "land\n"
      "rtl\n");
  CHECK_EQ(items.size(), std::size_t{5});
  CHECK_EQ(items[0].command, std::uint16_t{22});
  CHECK_NEAR(items[0].z, 15.0, 1e-6);
  CHECK_EQ(items[1].command, std::uint16_t{16});
  CHECK_EQ(items[1].x, 473977420);
  CHECK_EQ(items[1].y, 85455940);
  CHECK_NEAR(items[1].param1, 5.0, 1e-6);
  CHECK_EQ(items[1].frame, std::uint8_t{6});
  CHECK_EQ(items[2].command, std::uint16_t{21});
  CHECK_EQ(items[3].x, 0);
  CHECK_EQ(items[4].command, std::uint16_t{20});
  CHECK_EQ(items[4].frame, std::uint8_t{2});
}

TEST(parse_reports_line_numbers_for_errors) {
  CHECK_EQ(error_line("takeoff 10\nfly 1 2 3\n"), 2);
  CHECK_EQ(error_line("takeoff\n"), 1);
  CHECK_EQ(error_line("# c\n\nwaypoint 91 0 10\n"), 3);
  CHECK_EQ(error_line("waypoint 10 181 10\n"), 1);
  CHECK_EQ(error_line("waypoint 10 20 abc\n"), 1);
  CHECK_EQ(error_line("waypoint 10 20 0\n"), 1);
  CHECK_EQ(error_line("takeoff 10\nrtl now\n"), 2);
  CHECK_EQ(error_line("land 10\n"), 1);
  CHECK_EQ(error_line("waypoint 1 2 10 -4\n"), 1);
}

TEST(parse_rejects_empty_mission) {
  CHECK_THROWS(parse("# nothing\n\n"));
  CHECK_THROWS(load_mission("/nonexistent/mission.txt"));
}

TEST(parse_rejects_oversized_mission) {
  std::string text;
  for (std::size_t i = 0; i <= kMaxMissionItems; ++i) text += "takeoff 5\n";
  CHECK_THROWS(parse(text));
}

TEST(uploader_completes_full_handshake) {
  MissionUploader up(1, 1);
  const auto first = up.start(three_items(), 0);
  CHECK_EQ(first.size(), std::size_t{1});
  CHECK_EQ(std::get<MissionCount>(first[0]).count, std::uint16_t{3});
  CHECK(up.state() == MissionUploader::State::Uploading);

  for (std::uint16_t seq = 0; seq < 3; ++seq) {
    const auto reply = up.on_message(MissionRequestInt{seq, 255, 190}, 1000);
    CHECK_EQ(reply.size(), std::size_t{1});
    const auto& item = std::get<MissionItemInt>(reply[0]);
    CHECK_EQ(item.seq, seq);
    CHECK_EQ(item.current, static_cast<std::uint8_t>(seq == 0 ? 1 : 0));
    CHECK_EQ(item.target_system, std::uint8_t{1});
  }
  CHECK(up.on_message(MissionAck{255, 190, 0}, 2000).empty());
  CHECK(up.state() == MissionUploader::State::Done);
  CHECK_EQ(up.sent(), std::size_t{3});
}

TEST(uploader_retries_then_times_out) {
  MissionUploader up(1, 1);
  (void)up.start(three_items(), 0);
  CHECK(up.poll(500'000).empty());
  std::uint64_t t = 0;
  for (int i = 0; i < MissionUploader::kMaxRetries; ++i) {
    t += MissionUploader::kRetryUs;
    const auto retry = up.poll(t);
    CHECK_EQ(retry.size(), std::size_t{1});
    CHECK(std::holds_alternative<MissionCount>(retry[0]));
  }
  t += MissionUploader::kRetryUs;
  CHECK(up.poll(t).empty());
  CHECK(up.state() == MissionUploader::State::Failed);
  CHECK(up.error().find("timed out") != std::string::npos);
}

TEST(uploader_resends_requested_item_on_retry) {
  MissionUploader up(1, 1);
  (void)up.start(three_items(), 0);
  (void)up.on_message(MissionRequestInt{0, 255, 190}, 10);
  const auto retry = up.poll(10 + MissionUploader::kRetryUs);
  CHECK(std::holds_alternative<MissionItemInt>(retry.at(0)));
}

TEST(uploader_fails_on_rejection_and_bad_request) {
  MissionUploader a(1, 1);
  (void)a.start(three_items(), 0);
  (void)a.on_message(MissionAck{255, 190, 1}, 5);
  CHECK(a.state() == MissionUploader::State::Failed);

  MissionUploader b(1, 1);
  (void)b.start(three_items(), 0);
  CHECK(b.on_message(MissionRequestInt{9, 255, 190}, 5).empty());
  CHECK(b.state() == MissionUploader::State::Failed);

  MissionUploader c(1, 1);
  CHECK(c.start({}, 0).empty());
  CHECK(c.state() == MissionUploader::State::Failed);
}

TEST(uploader_ignores_traffic_when_idle) {
  MissionUploader up(1, 1);
  CHECK(up.on_message(MissionRequestInt{0, 255, 190}, 0).empty());
  CHECK(up.poll(10'000'000).empty());
  CHECK(up.state() == MissionUploader::State::Idle);
}
