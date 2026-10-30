#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace mavgs {

struct OptionSpec {
  std::string name;        ///< without leading dashes
  std::string value_name;  ///< empty for boolean flags
  std::string help;
  std::string default_text;  ///< shown in --help, empty if none
};

/// Minimal long-option parser: `--name value`, `--name=value` and flags.
class ArgParser {
 public:
  ArgParser(std::string program, std::string summary, std::vector<OptionSpec> options);

  /// Throws std::invalid_argument for unknown options or missing values.
  void parse(int argc, const char* const* argv);

  [[nodiscard]] bool has(const std::string& name) const { return values_.count(name) != 0; }
  [[nodiscard]] std::string get(const std::string& name, const std::string& fallback = "") const;
  /// Numeric accessor with range validation; throws std::invalid_argument.
  [[nodiscard]] double get_number(const std::string& name, double fallback, double lo,
                                  double hi) const;
  [[nodiscard]] std::string usage() const;

 private:
  [[nodiscard]] const OptionSpec* find(const std::string& name) const;

  std::string program_;
  std::string summary_;
  std::vector<OptionSpec> options_;
  std::map<std::string, std::string> values_;
};

}  // namespace mavgs
