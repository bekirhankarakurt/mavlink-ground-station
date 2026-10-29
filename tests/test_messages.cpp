#include "check.hpp"
#include "mavgs/bytes.hpp"
#include "mavgs/messages.hpp"

using namespace mavgs;

namespace {

template <typename M>
M roundtrip(const M& message) {
  Frame frame;
  frame.msgid = M::kId;
  frame.payload = encode_payload(message);
  const auto decoded = decode(frame);
  CHECK(decoded.has_value());
  return std::get<M>(*decoded);
}

}  // namespace

TEST(writer_reader_are_little_endian_and_symmetric) {
  Writer w;
  w.put(std::uint16_t{0x1234});
  w.put(std::int32_t{-2});
  w.put(1.5F);
  CHECK_EQ(w.bytes()[0], std::uint8_t{0x34});
  CHECK_EQ(w.bytes()[1], std::uint8_t{0x12});
  const Bytes bytes = w.take();
  Reader r(bytes);
  CHECK_EQ(r.get<std::uint16_t>(), std::uint16_t{0x1234});
  CHECK_EQ(r.get<std::int32_t>(), -2);
  CHECK_NEAR(r.get_float(), 1.5, 1e-9);
}

TEST(reader_zero_fills_past_end) {
  const Bytes bytes = {0x01};
  Reader r(bytes);
  CHECK_EQ(r.get<std::uint16_t>(), std::uint16_t{1});
  CHECK_EQ(r.get<std::uint32_t>(), std::uint32_t{0});
}

TEST(all_messages_roundtrip) {
  CHECK_EQ(roundtrip(Heartbeat{7, 2, 12, 129, 4, 3}).custom_mode, std::uint32_t{7});
  const auto sys = roundtrip(SysStatus{1, 2, 3, 400, 15000, -1, -1, 7});
  CHECK_EQ(sys.voltage_battery, std::uint16_t{15000});
  CHECK_EQ(sys.current_battery, std::int16_t{-1});
  CHECK_EQ(sys.battery_remaining, std::int8_t{-1});
  CHECK_EQ(sys.drop_rate_comm, std::uint16_t{7});
  CHECK_NEAR(roundtrip(Attitude{1, 0.5F, -0.25F, 3.0F, 0, 0, 0}).yaw, 3.0, 1e-6);
  const auto gpi = roundtrip(GlobalPositionInt{1, -473977420, 85455940, -5000, 25000, -1, 2, -3, 65535});
  CHECK_EQ(gpi.lat, -473977420);
  CHECK_EQ(gpi.alt, -5000);
  CHECK_EQ(gpi.hdg, std::uint16_t{65535});
  CHECK_EQ(roundtrip(GpsRawInt{99999999999ULL, 1, 2, 3, 4, 5, 6, 7, 3, 12}).time_usec, 99999999999ULL);
  CHECK_EQ(roundtrip(VfrHud{1, 2, 3, 4, -5, 60}).heading, std::int16_t{-5});
  const auto cmd = roundtrip(CommandLong{1, 2, 3, 4, 5, 6, 7, 400, 1, 1, 2});
  CHECK_EQ(cmd.command, std::uint16_t{400});
  CHECK_NEAR(cmd.param7, 7.0, 1e-9);
  CHECK_EQ(cmd.confirmation, std::uint8_t{2});
  CHECK_EQ(roundtrip(CommandAck{22, 2}).result, std::uint8_t{2});
  CHECK_EQ(roundtrip(MissionCount{700, 1, 1}).count, std::uint16_t{700});
  CHECK_EQ(roundtrip(MissionRequestInt{9, 1, 1}).seq, std::uint16_t{9});
  CHECK_EQ(roundtrip(MissionAck{255, 190, 13}).type, std::uint8_t{13});
  CHECK_EQ(roundtrip(MissionCurrent{65535}).seq, std::uint16_t{65535});
  CHECK_EQ(roundtrip(Ping{5, 6, 7, 8}).seq, std::uint32_t{6});
  const auto item = roundtrip(MissionItemInt{1, 2, 3, 4, -5, 6, 7, 8, 16, 1, 1, 6, 0, 1});
  CHECK_EQ(item.x, -5);
  CHECK_EQ(item.command, std::uint16_t{16});
}

TEST(statustext_trims_and_truncates) {
  CHECK_EQ(roundtrip(StatusText{4, "hello"}).text, std::string("hello"));
  const std::string long_text(80, 'x');
  CHECK_EQ(roundtrip(StatusText{4, long_text}).text.size(), std::size_t{50});
}

TEST(legacy_mission_request_id_decodes_like_int_variant) {
  Frame frame;
  frame.msgid = MissionRequestInt::kLegacyId;
  frame.payload = {3, 0, 1, 1};
  const auto m = decode(frame);
  CHECK(m.has_value());
  CHECK_EQ(std::get<MissionRequestInt>(*m).seq, std::uint16_t{3});
}

TEST(decode_ignores_unsupported_ids) {
  Frame frame;
  frame.msgid = 22;  // PARAM_VALUE, valid MAVLink but not handled
  CHECK(!decode(frame).has_value());
}

TEST(message_id_matches_variant_alternative) {
  CHECK_EQ(message_id(Message{Heartbeat{}}), std::uint32_t{0});
  CHECK_EQ(message_id(Message{MissionItemInt{}}), std::uint32_t{73});
}
