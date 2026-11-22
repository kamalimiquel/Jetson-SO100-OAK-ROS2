#!/usr/bin/env bash
# Every check the repository can run without hardware, in one command, with one verdict.
#
# Fourteen suites were being launched by hand, which is how "I thought it was green" happens. This runs each
# one, keeps its last line, and prints a single table; the exit code is the number of failures.
#
#   tools/check_all.sh          # quiet: one line per suite
#   tools/check_all.sh -v       # the full output of every suite
#
# Suites that need a bench, a camera or MuJoCo are deliberately absent: a suite that silently passes when the
# hardware is missing is worse than no suite.

set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

VERBOSE=0
[ "${1:-}" = "-v" ] && VERBOSE=1

PY=${PY:-python3}
pass=0
fail=0
failed_names=()

run() {
  local name="$1"; shift
  local out rc
  out=$("$@" 2>&1); rc=$?
  local last
  last=$(printf '%s\n' "$out" | tail -1)
  if [ $rc -eq 0 ]; then
    pass=$((pass + 1))
    printf 'ok    %-34s %s\n' "$name" "${last:0:80}"
  else
    fail=$((fail + 1))
    failed_names+=("$name")
    printf 'FAIL  %-34s %s\n' "$name" "${last:0:80}"
  fi
  if [ $VERBOSE -eq 1 ]; then
    printf '%s\n' "$out" | sed 's/^/      /'
  fi
}

# the contract and its planner
run "command contract"      "$PY" tools/test_command_contract.py
run "skill plan"            "$PY" tools/test_skill_plan.py
run "joint merge"           "$PY" tools/test_joint_merge.py
run "arbiter"               "$PY" tools/test_arbiter.py
run "episode recorder"      "$PY" tools/test_episode_recorder.py
run "joint server"          "$PY" tools/test_joint_server.py

# what the arm physically does
run "joint scale"           "$PY" tools/test_joint_scale.py
run "deadband"              "$PY" tools/test_deadband.py
run "parked pose"           "$PY" tools/test_park.py

# the surfaces above the arm
run "manifest"              "$PY" tools/test_manifest.py
run "mcp endpoint"          "$PY" tools/test_mcp_server.py
run "virtual arm (sim)"     "$PY" tools/test_sim_arm.py
run "twin commands"         "$PY" tools/test_twin_commands.py
run "units manifest"        "$PY" tools/test_units_manifest.py

# the repository's own invariants
run "namespaces"            "$PY" tools/check_namespaces.py
run "transforms"            "$PY" tools/check_transforms.py

echo
if [ $fail -eq 0 ]; then
  echo "ALL GREEN — $pass suites"
  exit 0
fi
echo "FAILED $fail of $((pass + fail)): ${failed_names[*]}"
exit $fail
