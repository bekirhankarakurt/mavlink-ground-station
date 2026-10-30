#include "mavgs/cli.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace mavgs {

ArgParser::ArgParser(std::string program, std::string summary, std::vector<OptionSpec> options)
    : program_(std::move(program)), summary_(std::move(summary)), options_(std::move(options)) {}

const OptionSpec* ArgParser::find(const std::string& name) const {
  for (const auto& option : options_) {
    if (option.name == name) return &option;
  }
  return nullptr;
}

void ArgParser::parse(int argc, const char* const* argv) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.rfind("--", 0) != 0) throw std::invalid_argument("unexpected argument '" + arg + "'");
    arg.erase(0, 2);
    std::string value;
    bool has_value = false;
    const auto eq = arg.find('=');
    if (eq != std::string::npos) {
      value = arg.substr(eq + 1);
      arg.erase(eq);
      has_value = true;
    }
    const OptionSpec* spec = find(arg);
    if (spec == nullptr) throw std::invalid_argument("unknown option '--" + arg + "'");
    if (spec->value_name.empty()) {
      if (has_value) throw std::invalid_argument("option '--" + arg + "' takes no value");
      values_[arg] = "";
      continue;
    }
    if (!has_value) {
      if (i + 1 >= argc) throw std::invalid_argument("option '--" + arg + "' needs a value");
      value = argv[++i];
    }
    values_[arg] = value;
  }
}

std::string ArgParser::get(const std::string& name, const std::string& fallback) const {
  const auto it = values_.find(name);
  return it == values_.end() ? fallback : it->second;
}

double ArgParser::get_number(const std::string& name, double fallback, double lo, double hi) const {
  const auto it = values_.find(name);
  if (it == values_.end()) return fallback;
  char* end = nullptr;
  const double value = std::strtod(it->second.c_str(), &end);
  if (it->second.empty() || *end != '\0' || !std::isfinite(value) || value < lo || value > hi) {
    std::ostringstream msg;
    msg << "option '--" << name << "' must be a number in [" << lo << ", " << hi << "], got '"
        << it->second << "'";
    throw std::invalid_argument(msg.str());
  }
  return value;
}

std::string ArgParser::usage() const {
  std::ostringstream out;
  out << summary_ << "\n\nUsage: " << program_ << " [options]\n\nOptions:\n";
  for (const auto& option : options_) {
    std::string left = "  --" + option.name;
    if (!option.value_name.empty()) left += " " + option.value_name;
    out << left << std::string(left.size() < 28 ? 28 - left.size() : 1, ' ') << option.help;
    if (!option.default_text.empty()) out << " (default: " << option.default_text << ")";
    out << "\n";
  }
  return out.str();
}

}  // namespace mavgs
