#include <cstdio>

#include "check.hpp"

int main() {
  for (const auto& test : tiny::registry()) {
    const int before = tiny::failures();
    test.fn();
    std::printf("[%s] %s\n", tiny::failures() == before ? " ok " : "FAIL", test.name);
  }
  if (tiny::failures() != 0) {
    std::printf("%d check(s) failed\n", tiny::failures());
    return 1;
  }
  return 0;
}
