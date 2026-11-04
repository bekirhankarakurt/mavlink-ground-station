# Contributing

Thanks for your interest. Bug reports, fixes and focused features are welcome.

## Build and test

```bash
cmake -S . -B build -DMAVGS_WERROR=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Before opening a pull request, also run the sanitizer build:

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DMAVGS_WERROR=ON -DMAVGS_SANITIZE=ON
cmake --build build-asan --parallel
ctest --test-dir build-asan --output-on-failure
```

CI additionally runs GCC and Clang, `cppcheck`, `clang-tidy` (configured in `.clang-tidy`), coverage and CodeQL.

## Style

- Modern C++17, RAII, no raw `new`/`delete`, const-correct, `[[nodiscard]]` on results that matter, `enum class`.
- Zero warnings with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`.
- No dependencies beyond the standard library and POSIX.
- No stubs, TODO comments or dead code. New behavior comes with a test; hardware-facing code needs a
  simulation or replay path so it can be tested without hardware.
- Keep MAVLink messages in `messages.hpp`/`messages.cpp`; verify wire layouts and `CRC_EXTRA` against a
  reference implementation (for example `pymavlink`) and add a golden frame to `tests/test_frame.cpp`.

## Commits and pull requests

- Small, logical commits with plain imperative messages, e.g. `Add MISSION_CLEAR_ALL support`.
- One topic per pull request; describe what changed and how you tested it.
- Update `README.md` and `CHANGELOG.md` (under "Unreleased") when behavior or options change.
