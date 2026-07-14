#!/usr/bin/env bash
# Contract test for access-computer plugin (AC-003).
# Covers: manifest 1.1, three-scenario validate+dry-run, SAR negative, exit codes.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
EXE="$BUILD/access-computer"
MANIFEST="$ROOT/configs/plugins/access-computer.json"
WORK_BASE="/tmp/mp_contract_access_computer_$$"

if [[ ! -x "$EXE" ]]; then
  echo "missing executable: $EXE" >&2
  echo "hint: meson compile -C build, or BUILD=/path/to/build $0" >&2
  exit 1
fi

pass=0
fail=0

assert_eq() {
  local name="$1" got="$2" want="$3"
  if [[ "$got" == "$want" ]]; then
    echo "  OK  $name (got=$got)"
    pass=$((pass + 1))
  else
    echo "  FAIL $name: got=$got want=$want" >&2
    fail=$((fail + 1))
  fi
}

assert_json_field() {
  local name="$1" json="$2" field="$3" want="$4"
  local got
  got="$(python3 -c "import json,sys; d=json.loads(sys.argv[1]); print(d.get(sys.argv[2],''))" "$json" "$field")"
  assert_eq "$name" "$got" "$want"
}

run_expect_exit() {
  local name="$1" want_exit="$2"
  shift 2
  set +e
  "$@" >/tmp/ac_contract_out_$$ 2>/tmp/ac_contract_err_$$
  local got=$?
  set -e
  assert_eq "$name" "$got" "$want_exit"
}

echo "==> [1] manifest matches on-disk + schema_version 1.1"
python3 - <<PY
import json, subprocess, sys
root = "$ROOT"
exe = "$EXE"
on_disk = json.load(open(root + "/configs/plugins/access-computer.json"))
runtime = json.loads(subprocess.check_output([exe, "manifest", "--output", "json"], text=True))
keys = ["schema_version", "name", "executable", "version", "commands"]
for k in keys:
    if runtime.get(k) != on_disk.get(k):
        print(f"manifest mismatch on {k}: {runtime.get(k)!r} vs {on_disk.get(k)!r}", file=sys.stderr)
        sys.exit(1)
if on_disk.get("schema_version") != "1.1":
    print(f"expected schema_version 1.1, got {on_disk.get('schema_version')!r}", file=sys.stderr)
    sys.exit(1)
caps = on_disk.get("capabilities") or {}
if caps.get("kind") != "compute":
    print(f"expected capabilities.kind=compute, got {caps.get('kind')!r}", file=sys.stderr)
    sys.exit(1)
produces = caps.get("produces") or []
if "WindowSet" not in produces:
    print(f"expected produces to include WindowSet, got {produces!r}", file=sys.stderr)
    sys.exit(1)
print("  OK  manifest + WindowSet/compute")
PY
pass=$((pass + 1))

validate_ok() {
  local name="$1" sample="$2"
  echo "==> validate $name"
  run_expect_exit "validate $name exit" 0 \
    "$EXE" validate --input "$ROOT/samples/$sample"
}

dry_run_ok() {
  local name="$1" sample="$2"
  local work="$WORK_BASE/$name"
  mkdir -p "$work"
  echo "==> dry-run $name"
  set +e
  out="$("$EXE" run --input "$ROOT/samples/$sample" --work-dir "$work" --dry-run --output json 2>/tmp/ac_contract_err_$$)"
  local got=$?
  set -e
  assert_eq "dry-run $name exit" "$got" "0"
  assert_json_field "dry-run $name status" "$out" "status" "dry_run"
  if [[ -f "$work/request.json" ]]; then
    echo "  OK  dry-run $name request.json"
    pass=$((pass + 1))
  else
    echo "  FAIL dry-run $name missing request.json" >&2
    fail=$((fail + 1))
  fi
}

validate_ok "RSA" "remote_sensing_access.json"
dry_run_ok "RSA" "remote_sensing_access.json"

validate_ok "AE" "attitude_estimation.json"
dry_run_ok "AE" "attitude_estimation.json"

validate_ok "DL" "downlink_window.json"
dry_run_ok "DL" "downlink_window.json"

echo "==> [8] SAR unsupported negative (exit 2)"
run_expect_exit "SAR validate exit" 2 \
  "$EXE" validate --input "$ROOT/samples/sar_unsupported.json"

echo "==> [9] validate missing --input (usage/validation)"
set +e
"$EXE" validate >/dev/null 2>&1
got=$?
set -e
if [[ "$got" == "2" || "$got" == "64" ]]; then
  echo "  OK  missing-input exit ($got)"
  pass=$((pass + 1))
else
  echo "  FAIL missing-input exit: got=$got want=2|64" >&2
  fail=$((fail + 1))
fi

rm -rf "$WORK_BASE"

echo "==> summary: pass=$pass fail=$fail"
if [[ "$fail" -ne 0 ]]; then
  exit 1
fi
echo "access-computer contract test passed"
