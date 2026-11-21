"""What the joint numbers mean, written down and checked at boot.

The failure this prevents is the quiet one. `SO100FollowerConfig` chooses between two unit systems
for the five body joints (`use_degrees=True` → degrees; `False` → a −100…100 fraction of the
calibrated travel) and always gives the jaw 0…100 %. Nothing raises if the SDK default changes, if a
joint is added to the map, or if the installed package reorders them: the arm moves, `deg2rad`
multiplies by π/180, the model looks plausible and every number downstream is wrong by a factor.

So the expectation lives here as data, and the joint server compares it with what the *installed*
SDK actually reports before it accepts a single command. On mismatch it refuses to start — a bench
that will not come up is a better outcome than a bench that moves in the wrong units.

`check()` is pure: `tools/test_units_manifest.py` feeds it good and bad manifests with no ROS, no
serial port and no lerobot installed.
"""

# Wire order of the array and of the ROS message. This is the one contract every layer shares.
JOINTS = (
    # name, bus id, model, normalisation mode reported by the SDK
    ("shoulder_pan", 1, "sts3215", "DEGREES"),
    ("shoulder_lift", 2, "sts3215", "DEGREES"),
    ("elbow_flex", 3, "sts3215", "DEGREES"),
    ("wrist_flex", 4, "sts3215", "DEGREES"),
    ("wrist_roll", 5, "sts3215", "DEGREES"),
    ("gripper", 6, "sts3215", "RANGE_0_100"),
)

# Units implied by the table above, for the record and for the tools that write a table.
UNIT_OF = {"DEGREES": "deg", "RANGE_0_100": "%", "RANGE_M100_100": "-100..100"}

# Keyword the bus methods must accept for a raw-tick read to be possible at all. Without it the
# audit table (raw ticks vs SDK vs external reference) cannot be produced.
REQUIRED_KWARG = "normalize"

EXPECTED_SDK_PREFIX = "0.7"     # recorded, warned about on drift, never a hard stop by itself


def observed_from(arm, config=None, sdk_version=None, signatures=None):
    """Build the observed manifest from an instantiated follower.

    `arm` is a lerobot follower with a `.bus.motors` mapping; `signatures` is a dict of
    {method_name: iterable of parameter names}. Kept dependency-free so a test can pass plain
    objects (or `types.SimpleNamespace`) instead of the real robot.
    """
    joints = []
    for name, motor in arm.bus.motors.items():
        joints.append({
            "name": name,
            "id": int(motor.id),
            "model": str(motor.model),
            "norm": getattr(motor.norm_mode, "name", str(motor.norm_mode)),
        })
    return {
        "sdk_version": sdk_version,
        "use_degrees": bool(getattr(config if config is not None else getattr(arm, "config", None),
                                   "use_degrees", None)),
        "joints": joints,
        "signatures": {k: sorted(v) for k, v in (signatures or {}).items()},
    }


def check(observed, absent=()):
    """Compare an observed manifest with the expectation. Returns (errors, warnings).

    `absent` names joints that are physically not on the bus this session (a servo unplugged on
    purpose, ABSENT_MOTORS). They are skipped in both directions — but only because the caller said so
    out loud; a motor that disappears without being declared is still an error.
    """
    errors, warnings = [], []
    absent = set(absent)

    if observed.get("use_degrees") is not True:
        errors.append("use_degrees is %r, expected True: the five body joints would be a -100..100 "
                      "fraction, not degrees" % (observed.get("use_degrees"),))

    want = [(n, i, m, m_) for n, i, m, m_ in JOINTS if n not in absent]
    got = [(j.get("name"), j.get("id"), j.get("model"), j.get("norm")) for j in observed.get("joints", [])]

    if len(got) != len(want):
        errors.append("%d motors reported, %d expected (%s)"
                      % (len(got), len(want), ", ".join(j[0] for j in want)))
    for name, mid, model, norm in want:
        match = [g for g in got if g[0] == name]
        if not match:
            errors.append("motor %r missing from the bus map" % name)
            continue
        gname, gid, gmodel, gnorm = match[0]
        if gid != mid:
            errors.append("%s: bus id %s, expected %d" % (name, gid, mid))
        if gmodel != model:
            errors.append("%s: model %r, expected %r" % (name, gmodel, model))
        if gnorm != norm:
            errors.append("%s: normalisation %r, expected %r — the unit of this joint is wrong"
                          % (name, gnorm, norm))
    order = [g[0] for g in got]
    expected_order = [j[0] for j in want]          # absent joints removed, order preserved
    if order and order != expected_order:
        errors.append("wire order %s, expected %s (the array has no names; order is the contract)"
                      % (order, expected_order))

    sig = observed.get("signatures") or {}
    for method in ("read", "write", "sync_write"):
        params = sig.get(method)
        if params is None:
            warnings.append("%s: signature not inspected — raw-tick reads are unverified" % method)
        elif REQUIRED_KWARG not in params:
            errors.append("%s() has no %r keyword: raw ticks cannot be read for the audit table"
                          % (method, REQUIRED_KWARG))

    version = observed.get("sdk_version")
    if not version:
        warnings.append("SDK version unknown")
    elif not str(version).startswith(EXPECTED_SDK_PREFIX):
        warnings.append("SDK version %s, last audited %s.x — re-run the unit table" % (version,
                                                                                       EXPECTED_SDK_PREFIX))
    return errors, warnings


def describe(observed):
    """One line per joint for a log or the ledger: name, id, model, mode, unit."""
    lines = []
    for j in observed.get("joints", []):
        lines.append("%-14s id=%s %-8s %-12s %s" % (j.get("name"), j.get("id"), j.get("model"),
                                                    j.get("norm"), UNIT_OF.get(j.get("norm"), "?")))
    return lines
