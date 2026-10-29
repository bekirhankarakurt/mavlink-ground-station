#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace mavgs {

using Bytes = std::vector<std::uint8_t>;

/// Little-endian payload writer.
class Writer {
 public:
  template <typename T>
  void put(T value) {
    static_assert(std::is_integral_v<T>, "integral types only");
    using U = std::make_unsigned_t<T>;
    const auto u = static_cast<std::uint64_t>(static_cast<U>(value));
    for (std::size_t i = 0; i < sizeof(T); ++i) {
      bytes_.push_back(static_cast<std::uint8_t>((u >> (8 * i)) & 0xFFu));
    }
  }

  void put(float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    put(bits);
  }

  /// Writes `text` into a fixed-size, zero-padded char array of `size` bytes.
  void put_chars(const std::string& text, std::size_t size) {
    for (std::size_t i = 0; i < size; ++i) {
      bytes_.push_back(i < text.size() ? static_cast<std::uint8_t>(text[i]) : std::uint8_t{0});
    }
  }

  [[nodiscard]] const Bytes& bytes() const { return bytes_; }
  [[nodiscard]] Bytes take() { return std::move(bytes_); }

 private:
  Bytes bytes_;
};

/// Little-endian payload reader. Reads past the end yield zero, which is how
/// MAVLink v2 zero-truncated payloads are expanded.
class Reader {
 public:
  explicit Reader(const Bytes& bytes) : bytes_(bytes) {}

  template <typename T>
  [[nodiscard]] T get() {
    static_assert(std::is_integral_v<T>, "integral types only");
    using U = std::make_unsigned_t<T>;
    std::uint64_t acc = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
      acc |= static_cast<std::uint64_t>(byte_at(pos_ + i)) << (8 * i);
    }
    pos_ += sizeof(T);
    return static_cast<T>(static_cast<U>(acc));
  }

  [[nodiscard]] float get_float() {
    const auto bits = get<std::uint32_t>();
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof value);
    return value;
  }

  /// Reads a fixed-size char array and trims at the first NUL.
  [[nodiscard]] std::string get_chars(std::size_t size) {
    std::string out;
    for (std::size_t i = 0; i < size; ++i) {
      const char c = static_cast<char>(byte_at(pos_ + i));
      if (c == '\0') break;
      out.push_back(c);
    }
    pos_ += size;
    return out;
  }

 private:
  [[nodiscard]] std::uint8_t byte_at(std::size_t index) const {
    return index < bytes_.size() ? bytes_[index] : std::uint8_t{0};
  }

  const Bytes& bytes_;
  std::size_t pos_ = 0;
};

}  // namespace mavgs
