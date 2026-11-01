# Security Policy

## Supported versions

| Version | Supported |
|---|---|
| 1.x | yes |
| < 1.0 | no |

## Reporting a vulnerability

Please do not open a public issue for security problems. Report them privately through
[GitHub Security Advisories](https://github.com/bekirhankarakurt/mavlink-ground-station/security/advisories/new)
("Report a vulnerability" on the repository's Security tab).

Include the affected version, a description of the impact and, if possible, a reproducer
(for example a crafted MAVLink frame or `.tlog` file). You can expect an acknowledgement
within a few days and a fix or mitigation plan once the report is confirmed.

## Scope notes

- The station and simulator parse untrusted bytes from UDP sockets and `.tlog` files; memory-safety
  or denial-of-service bugs in that path are in scope.
- MAVLink 2 message signatures are skipped, not verified. A link is therefore not authenticated; run
  it only on trusted networks. This is a documented limitation, not a vulnerability.
