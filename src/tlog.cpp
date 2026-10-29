#include "mavgs/tlog.hpp"

#include <iterator>
#include <stdexcept>

#include "mavgs/frame.hpp"

namespace mavgs {

TlogWriter::TlogWriter(const std::string& path) : out_(path, std::ios::binary | std::ios::trunc) {
  if (!out_) throw std::runtime_error("cannot open log for writing: " + path);
}

void TlogWriter::append(std::uint64_t timestamp_us, const Bytes& frame) {
  char stamp[8];
  for (int i = 0; i < 8; ++i) {
    stamp[i] = static_cast<char>((timestamp_us >> (8 * (7 - i))) & 0xFFu);
  }
  out_.write(stamp, sizeof stamp);
  out_.write(reinterpret_cast<const char*>(frame.data()),
             static_cast<std::streamsize>(frame.size()));
  out_.flush();
  ++records_;
}

std::vector<TlogRecord> parse_tlog(const Bytes& data) {
  std::vector<TlogRecord> records;
  std::size_t pos = 0;
  while (data.size() - pos > 8) {
    const std::uint8_t* frame = data.data() + pos + 8;
    const std::size_t avail = data.size() - pos - 8;
    const auto length = frame_length(frame, avail);
    if (!length || *length > avail) break;

    TlogRecord record;
    for (std::size_t i = 0; i < 8; ++i) {
      record.timestamp_us = (record.timestamp_us << 8) | data[pos + i];
    }
    record.frame.assign(frame, frame + *length);
    records.push_back(std::move(record));
    pos += 8 + *length;
  }
  return records;
}

std::vector<TlogRecord> read_tlog(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open log for reading: " + path);
  const Bytes data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return parse_tlog(data);
}

}  // namespace mavgs
