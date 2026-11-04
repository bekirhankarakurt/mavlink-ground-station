# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-11-04

### Added
- MAVLink v2 frame parser and serializer with X.25 CRC and `CRC_EXTRA` validation for the common dialect.
- Typed codecs for HEARTBEAT, SYS_STATUS, PING, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD,
  COMMAND_LONG/ACK, MISSION_COUNT/REQUEST/REQUEST_INT/ITEM_INT/ACK/CURRENT and STATUSTEXT.
- `mavgcs` terminal ground station: live dashboard and log output, arm/disarm/takeoff/land/RTL/mission
  start with ACK tracking and retries, scripted flights (`--script`).
- Link-quality statistics: packet loss from sequence gaps, message and byte rates, CRC errors, ping latency.
- Mission upload from a plain-text waypoint file with timeout and retry.
- Telemetry recording and replay in the QGroundControl-compatible `.tlog` layout.
- `mavsim` vehicle simulator with PX4-style modes, mission execution and a deterministic packet-loss model.
- Unit tests, simulator-backed station tests and a UDP end-to-end test, run through CTest.

[1.0.0]: https://github.com/bekirhankarakurt/mavlink-ground-station/releases/tag/v1.0.0
