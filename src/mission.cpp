#include "mavgs/mission.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "mavgs/enums.hpp"

namespace mavgs {

namespace {

double parse_number(const std::string& token, int line, const char* what) {
  char* end = nullptr;
  const double value = std::strtod(token.c_str(), &end);
  if (token.empty() || *end != '\0' || !std::isfinite(value)) {
    throw MissionParseError(line, std::string("invalid ") + what + " '" + token + "'");
  }
  return value;
}

double parse_ranged(const std::string& token, int line, const char* what, double lo, double hi) {
  const double value = parse_number(token, line, what);
  if (value < lo || value > hi) {
    throw MissionParseError(line, std::string(what) + " " + token + " out of range [" +
                                      std::to_string(static_cast<int>(lo)) + ", " +
                                      std::to_string(static_cast<int>(hi)) + "]");
  }
  return value;
}

std::int32_t to_e7(double degrees) {
  return static_cast<std::int32_t>(std::llround(degrees * 1e7));
}

void expect_args(const std::vector<std::string>& t, std::size_t min, std::size_t max, int line,
                 const char* usage) {
  const std::size_t args = t.size() - 1;
  if (args < min || args > max) {
    throw MissionParseError(line, "usage: " + std::string(usage));
  }
}

}  // namespace

std::vector<MissionItem> parse_mission(std::istream& in) {
  std::vector<MissionItem> items;
  std::string text;
  int line = 0;
  while (std::getline(in, text)) {
    ++line;
    if (const auto hash = text.find('#'); hash != std::string::npos) text.resize(hash);
    std::istringstream words(text);
    std::vector<std::string> t;
    for (std::string w; words >> w;) t.push_back(w);
    if (t.empty()) continue;
    std::transform(t[0].begin(), t[0].end(), t[0].begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    MissionItem item;
    item.frame = static_cast<std::uint8_t>(MavFrame::GlobalRelativeAltInt);
    if (t[0] == "takeoff") {
      expect_args(t, 1, 1, line, "takeoff <alt_m>");
      item.command = static_cast<std::uint16_t>(MavCmd::NavTakeoff);
      item.z = static_cast<float>(parse_ranged(t[1], line, "altitude", 1, 10000));
    } else if (t[0] == "waypoint") {
      expect_args(t, 3, 4, line, "waypoint <lat> <lon> <alt_m> [hold_s]");
      item.command = static_cast<std::uint16_t>(MavCmd::NavWaypoint);
      item.x = to_e7(parse_ranged(t[1], line, "latitude", -90, 90));
      item.y = to_e7(parse_ranged(t[2], line, "longitude", -180, 180));
      item.z = static_cast<float>(parse_ranged(t[3], line, "altitude", 1, 10000));
      if (t.size() == 5) {
        item.param1 = static_cast<float>(parse_ranged(t[4], line, "hold time", 0, 3600));
      }
    } else if (t[0] == "land") {
      if (t.size() != 1 && t.size() != 3) throw MissionParseError(line, "usage: land [<lat> <lon>]");
      item.command = static_cast<std::uint16_t>(MavCmd::NavLand);
      if (t.size() == 3) {
        item.x = to_e7(parse_ranged(t[1], line, "latitude", -90, 90));
        item.y = to_e7(parse_ranged(t[2], line, "longitude", -180, 180));
      }
    } else if (t[0] == "rtl") {
      expect_args(t, 0, 0, line, "rtl");
      item.command = static_cast<std::uint16_t>(MavCmd::NavReturnToLaunch);
      item.frame = static_cast<std::uint8_t>(MavFrame::Mission);
    } else {
      throw MissionParseError(line, "unknown command '" + t[0] + "'");
    }
    items.push_back(item);
    if (items.size() > kMaxMissionItems) {
      throw MissionParseError(line, "too many items (max " + std::to_string(kMaxMissionItems) + ")");
    }
  }
  if (items.empty()) throw MissionParseError(line, "mission has no items");
  return items;
}

std::vector<MissionItem> load_mission(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open mission file: " + path);
  return parse_mission(in);
}

MissionItemInt to_wire(const MissionItem& item, std::uint16_t seq, std::uint8_t target_system,
                       std::uint8_t target_component) {
  MissionItemInt m;
  m.param1 = item.param1;
  m.param2 = item.param2;
  m.param3 = item.param3;
  m.param4 = item.param4;
  m.x = item.x;
  m.y = item.y;
  m.z = item.z;
  m.seq = seq;
  m.command = item.command;
  m.target_system = target_system;
  m.target_component = target_component;
  m.frame = item.frame;
  m.current = seq == 0 ? 1 : 0;
  m.autocontinue = 1;
  return m;
}

void MissionUploader::fail(std::string reason) {
  state_ = State::Failed;
  error_ = std::move(reason);
}

std::vector<Message> MissionUploader::start(std::vector<MissionItem> items, std::uint64_t now_us) {
  items_ = std::move(items);
  sent_ = 0;
  retries_ = 0;
  error_.clear();
  if (items_.empty() || items_.size() > kMaxMissionItems) {
    fail("mission must contain 1.." + std::to_string(kMaxMissionItems) + " items");
    return {};
  }
  state_ = State::Uploading;
  last_tx_us_ = now_us;
  last_tx_ = MissionCount{static_cast<std::uint16_t>(items_.size()), target_system_, target_component_};
  return {last_tx_};
}

std::vector<Message> MissionUploader::on_message(const Message& message, std::uint64_t now_us) {
  if (state_ != State::Uploading) return {};

  if (const auto* request = std::get_if<MissionRequestInt>(&message)) {
    if (request->seq >= items_.size()) {
      fail("vehicle requested item " + std::to_string(request->seq) + " of " +
           std::to_string(items_.size()));
      return {};
    }
    sent_ = std::max<std::size_t>(sent_, request->seq + std::size_t{1});
    retries_ = 0;
    last_tx_us_ = now_us;
    last_tx_ = to_wire(items_[request->seq], request->seq, target_system_, target_component_);
    return {last_tx_};
  }
  if (const auto* ack = std::get_if<MissionAck>(&message)) {
    if (ack->type == static_cast<std::uint8_t>(MissionResult::Accepted)) {
      sent_ = items_.size();
      state_ = State::Done;
    } else {
      fail("vehicle rejected mission (result " + std::to_string(ack->type) + ")");
    }
  }
  return {};
}

std::vector<Message> MissionUploader::poll(std::uint64_t now_us) {
  if (state_ != State::Uploading || now_us - last_tx_us_ < kRetryUs) return {};
  if (retries_ >= kMaxRetries) {
    fail("upload timed out");
    return {};
  }
  ++retries_;
  last_tx_us_ = now_us;
  return {last_tx_};
}

}  // namespace mavgs
