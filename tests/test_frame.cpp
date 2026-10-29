#include <string>

#include "check.hpp"
#include "mavgs/crc.hpp"
#include "mavgs/frame.hpp"
#include "mavgs/messages.hpp"

using namespace mavgs;

namespace {

Bytes from_hex(const std::string& hex) {
  Bytes out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

// Reference frames produced by pymavlink 2.4.50 (common dialect), sysid 1 / compid 1.
struct Golden {
  const char* name;
  std::uint8_t seq;
  const char* hex;
  Message message;
};

std::vector<Golden> goldens() {
  std::vector<Golden> g;
  g.push_back({"HEARTBEAT", 0, "fd09000000010100000000030400020c810403ba4c",
               Heartbeat{0x040300, 2, 12, 129, 4, 3}});
  g.push_back({"SYS_STATUS", 1,
               "fd1f00000101010100002f0000002f0000002f000000fa00b83d3e030300000000000000000000004ad788",
               SysStatus{0x2f, 0x2f, 0x2f, 250, 15800, 830, 74, 3}});
  g.push_back({"ATTITUDE", 2,
               "fd1c00000201011e000039300000cdcccc3dcdcc4cbe0000c03f0ad7233c0ad7a33c8fc2f53c821f",
               Attitude{12345, 0.1F, -0.2F, 1.5F, 0.01F, 0.02F, 0.03F}});
  g.push_back({"GLOBAL_POSITION_INT", 3,
               "fd1c0000030101210000881300004c52401c44f41705b0d40700a86100006400ceff88fffc21b629",
               GlobalPositionInt{5000, 473977420, 85455940, 513200, 25000, 100, -50, -120, 8700}});
  g.push_back({"GPS_RAW_INT", 4,
               "fd1e000004010118000015cd5b07000000004c52401c44f41705b0d4070050007800f4012823030c2894",
               GpsRawInt{123456789, 473977420, 85455940, 513200, 80, 120, 500, 9000, 3, 12}});
  g.push_back({"VFR_HUD", 5, "fd1300000501014a00000000b0403333a340cd4c00449a99993f570034ff8c",
               VfrHud{5.5F, 5.1F, 513.2F, 1.2F, 87, 52}});
  g.push_back({"COMMAND_LONG", 6,
               "fd2000000601014c00000000803f000000000000000000000000000000000000000000000000900101014e45",
               CommandLong{1, 0, 0, 0, 0, 0, 0, 400, 1, 1, 0}});
  g.push_back({"COMMAND_ACK", 7, "fd0200000701014d0000900142b1", CommandAck{400, 0}});
  g.push_back({"MISSION_COUNT", 8, "fd0400000801012c000005000101f69d", MissionCount{5, 1, 1}});
  g.push_back({"MISSION_REQUEST_INT", 9, "fd040000090101330000030001017594",
               MissionRequestInt{3, 1, 1}});
  g.push_back({"MISSION_ITEM_INT", 10,
               "fd2500000a01014900000000a0400000000000000000000000004c52401c44f417050000f041020010000101060001b0a1",
               MissionItemInt{5.0F, 0, 0, 0, 473977420, 85455940, 30.0F, 2, 16, 1, 1, 6, 0, 1}});
  g.push_back({"MISSION_ACK", 11, "fd0200000b01012f0000ffbe11fd", MissionAck{255, 190, 0}});
  g.push_back({"MISSION_CURRENT", 12, "fd0100000c01012a00000242c0", MissionCurrent{2}});
  g.push_back({"STATUSTEXT", 13,
               "fd1100000d0101fd00000654616b656f666620646574656374656414db",
               StatusText{6, "Takeoff detected"}});
  g.push_back({"PING", 14, "fd0900000e0101040000b168de3a000000002a1e65", Ping{987654321, 42, 0, 0}});
  return g;
}

Bytes pack_with_seq(const Message& m, std::uint8_t seq) {
  Frame f;
  f.seq = seq;
  f.sysid = 1;
  f.compid = 1;
  f.msgid = message_id(m);
  f.payload = encode_payload(m);
  return serialize(f);
}

}  // namespace

TEST(crc_matches_x25_check_value) {
  const std::string s = "123456789";
  CHECK_EQ(crc_calculate(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()),
           std::uint16_t{0x6F91});
}

TEST(crc_extra_lookup) {
  CHECK_EQ(*crc_extra(0), std::uint8_t{50});
  CHECK_EQ(*crc_extra(76), std::uint8_t{152});
  CHECK(!crc_extra(60000).has_value());
}

TEST(serialize_matches_pymavlink_reference) {
  for (const auto& g : goldens()) {
    const Bytes got = pack_with_seq(g.message, g.seq);
    if (got != from_hex(g.hex)) tiny::fail(__FILE__, __LINE__, std::string("serialize mismatch: ") + g.name);
  }
}

TEST(parse_pymavlink_reference_frames) {
  for (const auto& g : goldens()) {
    Parser parser;
    const auto frames = parser.feed(from_hex(g.hex));
    if (frames.size() != 1) {
      tiny::fail(__FILE__, __LINE__, std::string("expected one frame: ") + g.name);
      continue;
    }
    CHECK_EQ(frames[0].frame.seq, g.seq);
    CHECK_EQ(frames[0].frame.msgid, message_id(g.message));
    CHECK(frames[0].verified);
    const auto decoded = decode(frames[0].frame);
    CHECK(decoded.has_value());
    if (decoded) CHECK_EQ(encode_payload(*decoded), encode_payload(g.message));
  }
}

TEST(parser_handles_byte_by_byte_feed) {
  Parser parser;
  std::size_t count = 0;
  for (const auto& g : goldens()) {
    for (const std::uint8_t b : from_hex(g.hex)) count += parser.feed(&b, 1).size();
  }
  CHECK_EQ(count, goldens().size());
  CHECK_EQ(parser.stats().crc_errors, std::uint64_t{0});
}

TEST(parser_rejects_corrupt_crc_and_resyncs) {
  Bytes stream = from_hex("00ff12");  // leading garbage
  Bytes bad = from_hex(goldens()[0].hex);
  bad[12] = static_cast<std::uint8_t>(bad[12] ^ 0x55);
  stream.insert(stream.end(), bad.begin(), bad.end());
  const Bytes good = from_hex(goldens()[7].hex);
  stream.insert(stream.end(), good.begin(), good.end());

  Parser parser;
  const auto frames = parser.feed(stream);
  CHECK_EQ(frames.size(), std::size_t{1});
  CHECK_EQ(frames[0].frame.msgid, std::uint32_t{77});
  CHECK_EQ(parser.stats().crc_errors, std::uint64_t{1});
  CHECK(parser.stats().discarded_bytes >= 3);
}

TEST(parser_skips_signature_bytes) {
  Bytes frame = from_hex(goldens()[0].hex);
  // Re-sign: set the incompat flag and recompute the CRC, then append 13 signature bytes.
  frame[2] = kIncompatSigned;
  const std::size_t len = frame[1];
  std::uint16_t crc = crc_calculate(frame.data() + 1, kHeaderSize - 1 + len);
  crc = crc_accumulate(50, crc);
  frame[kHeaderSize + len] = static_cast<std::uint8_t>(crc & 0xFF);
  frame[kHeaderSize + len + 1] = static_cast<std::uint8_t>(crc >> 8);
  frame.insert(frame.end(), kSignatureSize, 0xAB);
  const Bytes next = from_hex(goldens()[7].hex);
  frame.insert(frame.end(), next.begin(), next.end());

  Parser parser;
  const auto frames = parser.feed(frame);
  CHECK_EQ(frames.size(), std::size_t{2});
  CHECK_EQ(frames[0].raw.size(), std::size_t{kHeaderSize + len + kChecksumSize + kSignatureSize});
}

TEST(parser_rejects_unknown_incompat_flags) {
  Bytes frame = from_hex(goldens()[0].hex);
  frame[2] = 0x02;
  Parser parser;
  CHECK(parser.feed(frame).empty());
  CHECK(parser.stats().bad_flags >= 1);
}

TEST(unknown_message_id_passes_unverified) {
  Bytes frame = {kMagicV2, 1, 0, 0, 7, 1, 1, 0x70, 0x17, 0x00, 0x42, 0x00, 0x00};  // id 5998
  Parser parser;
  const auto frames = parser.feed(frame);
  CHECK_EQ(frames.size(), std::size_t{1});
  CHECK(!frames[0].verified);
  CHECK(!decode(frames[0].frame).has_value());
}

TEST(serialize_truncates_zero_payload_but_keeps_one_byte) {
  Frame f;
  f.msgid = MissionCurrent::kId;
  f.payload = {0, 0};
  const Bytes wire = serialize(f);
  CHECK_EQ(wire[1], std::uint8_t{1});
  Parser parser;
  const auto frames = parser.feed(wire);
  CHECK_EQ(frames.size(), std::size_t{1});
  const auto m = decode(frames[0].frame);
  CHECK(m.has_value());
  CHECK_EQ(std::get<MissionCurrent>(*m).seq, std::uint16_t{0});
}

TEST(serialize_rejects_unknown_id_and_oversize_payload) {
  Frame unknown;
  unknown.msgid = 60000;
  CHECK_THROWS(serialize(unknown));
  Frame big;
  big.msgid = 0;
  big.payload.assign(256, 1);
  CHECK_THROWS(serialize(big));
}

TEST(packer_increments_sequence_and_wraps) {
  Packer packer(1, 1);
  Parser parser;
  for (int i = 0; i < 258; ++i) {
    const auto frames = parser.feed(packer.pack(Heartbeat{}));
    CHECK_EQ(frames.size(), std::size_t{1});
    CHECK_EQ(frames[0].frame.seq, static_cast<std::uint8_t>(i & 0xFF));
  }
}
