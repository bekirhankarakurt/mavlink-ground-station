#pragma once

#include <termios.h>

namespace mavgs {

/// RAII raw-ish keyboard mode (no echo, no line buffering) for single-key
/// commands. Does nothing if stdin is not a terminal.
class KeyboardMode {
 public:
  KeyboardMode();
  ~KeyboardMode();
  KeyboardMode(const KeyboardMode&) = delete;
  KeyboardMode& operator=(const KeyboardMode&) = delete;
  KeyboardMode(KeyboardMode&&) = delete;
  KeyboardMode& operator=(KeyboardMode&&) = delete;

  /// Returns the next pressed key, or -1 if none is waiting.
  [[nodiscard]] int poll_key() const;
  [[nodiscard]] bool active() const { return active_; }

 private:
  termios saved_{};
  bool active_ = false;
};

}  // namespace mavgs
