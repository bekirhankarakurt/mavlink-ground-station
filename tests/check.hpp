#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace tiny {

struct Case {
  const char* name;
  void (*fn)();
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

inline int& failures() {
  static int count = 0;
  return count;
}

struct Registrar {
  Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline void fail(const char* file, int line, const std::string& what) {
  ++failures();
  std::fprintf(stderr, "%s:%d: %s\n", file, line, what.c_str());
}

}  // namespace tiny

#define TEST(name)                                          \
  static void name();                                       \
  static const ::tiny::Registrar registrar_##name{#name, name}; \
  static void name()

#define CHECK(cond) \
  do { if (!(cond)) ::tiny::fail(__FILE__, __LINE__, "CHECK failed: " #cond); } while (false)

#define CHECK_EQ(a, b)                                                                     \
  do {                                                                                     \
    const auto va_ = (a);                                                                  \
    const auto vb_ = (b);                                                                  \
    if (!(va_ == vb_)) ::tiny::fail(__FILE__, __LINE__, "CHECK_EQ failed: " #a " == " #b); \
  } while (false)

#define CHECK_NEAR(a, b, eps)                                                               \
  do {                                                                                      \
    const double va_ = static_cast<double>(a);                                              \
    const double vb_ = static_cast<double>(b);                                              \
    if (!(std::fabs(va_ - vb_) <= (eps)))                                                   \
      ::tiny::fail(__FILE__, __LINE__,                                                      \
                   "CHECK_NEAR failed: " #a " ~ " #b " (" + std::to_string(va_) + " vs " + \
                       std::to_string(vb_) + ")");                                          \
  } while (false)

#define CHECK_THROWS(expr)                                                       \
  do {                                                                           \
    bool threw_ = false;                                                         \
    try {                                                                        \
      (void)(expr);                                                              \
    } catch (const std::exception&) {                                            \
      threw_ = true;                                                             \
    }                                                                            \
    if (!threw_) ::tiny::fail(__FILE__, __LINE__, "CHECK_THROWS failed: " #expr); \
  } while (false)
