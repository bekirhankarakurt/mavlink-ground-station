#include "mavgs/script.hpp"

#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace mavgs {

std::vector<ScriptStep> parse_script(const std::string& text) {
  std::vector<ScriptStep> steps;
  std::istringstream in(text);
  for (std::string token; std::getline(in, token, ',');) {
    const auto colon = token.find(':');
    const std::string name = token.substr(0, colon);
    const std::string arg = colon == std::string::npos ? "" : token.substr(colon + 1);
    ScriptStep step;
    step.label = token;

    const auto number = [&](double lo, double hi) {
      char* end = nullptr;
      const double value = std::strtod(arg.c_str(), &end);
      if (arg.empty() || *end != '\0' || value < lo || value > hi) {
        throw std::invalid_argument("script step '" + token + "' needs a number in [" +
                                    std::to_string(static_cast<int>(lo)) + ", " +
                                    std::to_string(static_cast<int>(hi)) + "]");
      }
      return value;
    };
    const auto no_arg = [&] {
      if (colon != std::string::npos) throw std::invalid_argument("script step '" + token + "' takes no argument");
    };

    if (name == "arm") { no_arg(); step.kind = ScriptStep::Kind::Arm; }
    else if (name == "disarm") { no_arg(); step.kind = ScriptStep::Kind::Disarm; }
    else if (name == "land") { no_arg(); step.kind = ScriptStep::Kind::Land; }
    else if (name == "rtl") { no_arg(); step.kind = ScriptStep::Kind::Rtl; }
    else if (name == "start") { no_arg(); step.kind = ScriptStep::Kind::StartMission; }
    else if (name == "wait-disarmed") { no_arg(); step.kind = ScriptStep::Kind::WaitDisarmed; }
    else if (name == "takeoff") { step.kind = ScriptStep::Kind::Takeoff; step.value = number(1, 10000); }
    else if (name == "wait") { step.kind = ScriptStep::Kind::Wait; step.value = number(0, 86400); }
    else if (name == "mission") {
      if (arg.empty()) throw std::invalid_argument("script step 'mission' needs a file: mission:<file>");
      step.kind = ScriptStep::Kind::UploadMission;
      step.items = load_mission(arg);
    } else {
      throw std::invalid_argument("unknown script step '" + token + "'");
    }
    steps.push_back(std::move(step));
  }
  if (steps.empty()) throw std::invalid_argument("script is empty");
  return steps;
}

void ScriptRunner::next() {
  ++index_;
  issued_ = false;
}

void ScriptRunner::fail(const std::string& reason) {
  failed_ = true;
  error_ = "script step '" + steps_[index_].label + "': " + reason;
}

void ScriptRunner::advance(Station& station) {
  if (finished()) return;
  if (index_ == 0 && !issued_ && !station.vehicle_online()) return;  // wait for the link first

  const ScriptStep& step = steps_[index_];
  const auto now = station.now_us();

  switch (step.kind) {
    case ScriptStep::Kind::Arm:
    case ScriptStep::Kind::Disarm:
    case ScriptStep::Kind::Takeoff:
    case ScriptStep::Kind::Land:
    case ScriptStep::Kind::Rtl:
    case ScriptStep::Kind::StartMission: {
      if (!issued_) {
        bool sent = false;
        switch (step.kind) {
          case ScriptStep::Kind::Arm: sent = station.arm(); break;
          case ScriptStep::Kind::Disarm: sent = station.disarm(); break;
          case ScriptStep::Kind::Takeoff: sent = station.takeoff(step.value); break;
          case ScriptStep::Kind::Land: sent = station.land(); break;
          case ScriptStep::Kind::Rtl: sent = station.return_to_launch(); break;
          default: sent = station.start_mission(); break;
        }
        if (!sent) {
          fail("could not be sent");
          return;
        }
        issued_ = true;
        return;
      }
      switch (station.command_status()) {
        case CommandStatus::Accepted: next(); break;
        case CommandStatus::Rejected: fail("rejected by vehicle"); break;
        case CommandStatus::TimedOut: fail("no response from vehicle"); break;
        default: break;
      }
      break;
    }
    case ScriptStep::Kind::UploadMission:
      if (!issued_) {
        if (!station.upload_mission(step.items)) {
          fail("upload could not start");
          return;
        }
        issued_ = true;
        return;
      }
      if (station.uploader().state() == MissionUploader::State::Done) {
        next();
      } else if (station.uploader().state() == MissionUploader::State::Failed) {
        fail(station.uploader().error());
      }
      break;
    case ScriptStep::Kind::Wait:
      if (!issued_) {
        deadline_us_ = now + static_cast<std::uint64_t>(step.value * 1e6);
        issued_ = true;
      }
      if (now >= deadline_us_) next();
      break;
    case ScriptStep::Kind::WaitDisarmed:
      issued_ = true;
      saw_armed_ = saw_armed_ || station.vehicle().armed();
      if (saw_armed_ && !station.vehicle().armed()) {
        saw_armed_ = false;
        next();
      }
      break;
  }
}

}  // namespace mavgs
