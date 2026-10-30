#include "mavgs/terminal.hpp"

#include <poll.h>
#include <unistd.h>

namespace mavgs {

KeyboardMode::KeyboardMode() {
  if (isatty(STDIN_FILENO) == 0 || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
  termios raw = saved_;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
}

KeyboardMode::~KeyboardMode() {
  if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
}

int KeyboardMode::poll_key() const {
  if (!active_) return -1;
  pollfd pfd{STDIN_FILENO, POLLIN, 0};
  if (::poll(&pfd, 1, 0) <= 0) return -1;
  unsigned char c = 0;
  return ::read(STDIN_FILENO, &c, 1) == 1 ? c : -1;
}

}  // namespace mavgs
