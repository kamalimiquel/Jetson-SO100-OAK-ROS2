# Bruno collection — the MCP endpoint

Open this folder in [Bruno](https://usebruno.com) (*Open Collection* → `mcp/bruno`), pick the `local`
environment and edit `baseUrl` / `supervisorUrl` there. Nothing here is generated: each `.bru` is one request
with the body the endpoint expects, so the collection doubles as the endpoint's documentation.

| # | request | what it proves |
|---|---|---|
| 01 | `tools/list` | the ten tools and their schemas |
| 02 | `capabilities` | identity, epoch, skills, profiles |
| 03 | `get_state` | per-arm scale and deadband per joint, owner, torque |
| 04 | `move_joint_bounded` | degrees by name; refused with a sentence if the cell is not parked or torque is off |
| 05 | `set_gripper_aperture` | millimetres resolved by that arm's own curve |
| 06 | `solve_ik` | report only; never writes |
| 07 | `camera_snapshot` | JPEG with observation id, serial, stamp and hash |
| 08 | `detect_objects` | `MISSING` while there is no tracker — the honest answer |
| 09 | `get_goal` | what the journal says about the mission |
| 10 | `cancel_goal` | drops that mission's pending work |
| 11 | `stop` | the latch; the endpoint cannot lift it |
| 12–16 | supervisor `/stop`, `/resume`, `/cancel`, `/status`, `/journal` | the surface between agent and arm |

Expected answers with the cell idle: 04 and 05 stop at **409 torque off**, 06 needs the IK node running,
08 returns `MISSING`, and 12–16 answer even when no mission is up.

Variables: `arm` (`so100_cell` / `so100_cell_b`), `mission` (any string; it is the idempotency and journal
key), `baseUrl`, `supervisorUrl`.
