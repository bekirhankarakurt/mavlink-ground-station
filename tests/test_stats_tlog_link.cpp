#include <cstdio>
#include <fstream>
#include <thread>

#include "check.hpp"
#include "mavgs/frame.hpp"
#include "mavgs/link.hpp"
#include "mavgs/messages.hpp"
#include "mavgs/stats.hpp"
#include "mavgs/tlog.hpp"

using namespace mavgs;
using namespace std::chrono_literals;

namespace {

std::string temp_path(const char* name) {
  return (std::getenv("TMPDIR") != nullptr ? std::string(std::getenv("TMPDIR")) : std::string("/tmp")) +
         "/mavgs_test_" + name;
}

Bytes heartbeat_frame(std::uint8_t seq) {
  Packer p(1, 1);
  Bytes wire;
  for (int i = 0; i <= seq; ++i) wire = p.pack(Heartbeat{});
  return wire;
}

}  // namespace

TEST(stats_counts_sequence_gaps_across_wraparound) {
  LinkStats s;
  for (const int seq : {250, 251, 252, 255, 0, 1}) {
    s.on_frame(1, 1, static_cast<std::uint8_t>(seq), 20);
  }
  CHECK_EQ(s.received(), std::uint64_t{6});
  CHECK_EQ(s.lost(), std::uint64_t{2});  // 253 and 254
  CHECK_NEAR(s.loss_percent(), 25.0, 1e-9);
  CHECK_EQ(s.bytes(), std::uint64_t{120});
}

TEST(stats_tracks_sources_independently) {
  LinkStats s;
  s.on_frame(1, 1, 10, 10);
  s.on_frame(2, 1, 200, 10);
  s.on_frame(1, 1, 11, 10);
  s.on_frame(2, 1, 201, 10);
  CHECK_EQ(s.lost(), std::uint64_t{0});
}

TEST(stats_loss_is_zero_without_traffic) {
  CHECK_NEAR(LinkStats{}.loss_percent(), 0.0, 1e-12);
}

TEST(stats_rates_update_once_per_second) {
  LinkStats s;
  s.tick(0);
  for (int i = 0; i < 20; ++i) s.on_frame(1, 1, static_cast<std::uint8_t>(i), 50);
  s.tick(500'000);
  CHECK_NEAR(s.msg_rate(), 0.0, 1e-9);
  s.tick(2'000'000);
  CHECK_NEAR(s.msg_rate(), 10.0, 1e-9);
  CHECK_NEAR(s.byte_rate(), 500.0, 1e-9);
}

TEST(stats_latency_summary) {
  LinkStats s;
  CHECK(!s.has_latency());
  s.on_latency_sample(2000);
  s.on_latency_sample(6000);
  s.on_latency_sample(4000);
  CHECK_NEAR(s.latency_last_ms(), 4.0, 1e-9);
  CHECK_NEAR(s.latency_avg_ms(), 4.0, 1e-9);
  CHECK_NEAR(s.latency_min_ms(), 2.0, 1e-9);
  CHECK_NEAR(s.latency_max_ms(), 6.0, 1e-9);
}

TEST(tlog_roundtrip_and_qgc_layout) {
  const std::string path = temp_path("roundtrip.tlog");
  {
    TlogWriter w(path);
    w.append(0x0102030405060708ULL, heartbeat_frame(0));
    w.append(0x0102030405060709ULL, heartbeat_frame(1));
    CHECK_EQ(w.records(), std::uint64_t{2});
  }
  std::ifstream raw(path, std::ios::binary);
  char first[8];
  raw.read(first, 8);
  CHECK_EQ(static_cast<unsigned char>(first[0]), 0x01);  // big-endian timestamp
  CHECK_EQ(static_cast<unsigned char>(first[7]), 0x08);

  const auto records = read_tlog(path);
  CHECK_EQ(records.size(), std::size_t{2});
  CHECK_EQ(records[1].timestamp_us, 0x0102030405060709ULL);
  CHECK_EQ(records[1].frame, heartbeat_frame(1));
  std::remove(path.c_str());
}

TEST(tlog_ignores_truncated_tail_and_garbage) {
  Bytes data;
  for (int i = 0; i < 8; ++i) data.push_back(0);
  const Bytes frame = heartbeat_frame(0);
  data.insert(data.end(), frame.begin(), frame.end());
  Bytes truncated = data;
  truncated.insert(truncated.end(), data.begin(), data.end() - 4);
  CHECK_EQ(parse_tlog(truncated).size(), std::size_t{1});

  Bytes garbage = data;
  garbage.insert(garbage.end(), 12, 0x55);
  CHECK_EQ(parse_tlog(garbage).size(), std::size_t{1});
  CHECK(parse_tlog({}).empty());
}

TEST(tlog_reports_unopenable_paths) {
  CHECK_THROWS(TlogWriter("/nonexistent-dir/x.tlog"));
  CHECK_THROWS(read_tlog("/nonexistent-dir/x.tlog"));
}

TEST(parse_endpoint_accepts_and_rejects) {
  const auto ok = parse_endpoint("127.0.0.1:14550");
  CHECK(ok.has_value());
  CHECK_EQ(ok->port, std::uint16_t{14550});
  CHECK_EQ(parse_endpoint("localhost:5760")->host, std::string("127.0.0.1"));
  CHECK(!parse_endpoint("127.0.0.1").has_value());
  CHECK(!parse_endpoint("127.0.0.1:0").has_value());
  CHECK(!parse_endpoint("127.0.0.1:99999").has_value());
  CHECK(!parse_endpoint("example.invalid:80").has_value());
  CHECK(!parse_endpoint(":80").has_value());
}

TEST(udp_link_exchanges_datagrams_and_learns_peer) {
  UdpLink a(0);
  UdpLink b(0, Endpoint{"127.0.0.1", a.local_port()});
  CHECK(!a.has_peer());
  CHECK(!a.send({1}));  // nowhere to send yet

  CHECK(b.send({1, 2, 3}));
  Bytes got;
  CHECK(a.receive(got, 1000ms));
  CHECK_EQ(got, (Bytes{1, 2, 3}));
  CHECK(a.has_peer());

  CHECK(a.send({9}));  // reply goes to the sender
  CHECK(b.receive(got, 1000ms));
  CHECK_EQ(got, (Bytes{9}));
  CHECK(!b.receive(got, 10ms));
}

TEST(udp_link_reports_port_in_use) {
  UdpLink a(0);
  CHECK_THROWS(UdpLink(a.local_port()));
}

TEST(replay_link_delivers_in_order_with_timing) {
  std::vector<TlogRecord> records = {{1'000'000, heartbeat_frame(0)},
                                     {1'200'000, heartbeat_frame(1)}};
  ReplayLink link(records, 4.0, "mem");  // 200 ms gap -> 50 ms
  CHECK(link.is_replay());
  CHECK(!link.send({1}));
  Bytes got;
  CHECK(link.receive(got, 100ms));
  CHECK_EQ(got, records[0].frame);
  CHECK(!link.receive(got, 5ms));  // second record not due yet
  CHECK(link.receive(got, 200ms));
  CHECK_EQ(got, records[1].frame);
  CHECK(link.finished());
  CHECK(!link.receive(got, 1ms));
}
