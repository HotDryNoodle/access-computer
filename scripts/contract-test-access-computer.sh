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

echo "==> [10] AC-004 time_model details (RSA)"
set +e
out="$("$EXE" validate --input "$ROOT/samples/remote_sensing_access.json" --output json 2>/dev/null)"
got=$?
set -e
assert_eq "AC-004 RSA validate exit" "$got" "0"
printf '%s' "$out" | python3 -c '
import json,sys
d=json.load(sys.stdin)
tm=(d.get("details") or {}).get("time_model") or {}
for k in ("delta_prop_sec","compute_horizon_sec","working_time_sec"):
    if k not in tm:
        raise SystemExit("missing time_model.%s: %r" % (k, tm))
if abs(float(tm["delta_prop_sec"])) > 1e-6:
    raise SystemExit("RSA delta_prop expected 0, got %s" % tm["delta_prop_sec"])
'
echo "  OK  AC-004 RSA time_model"
pass=$((pass + 1))

echo "==> [10b] AC-004 AE delta_prop ~= 13020"
out="$("$EXE" validate --input "$ROOT/samples/attitude_estimation.json" --output json)"
printf '%s' "$out" | python3 -c '
import json,sys
d=json.load(sys.stdin)
tm=(d.get("details") or {}).get("time_model") or {}
dp=float(tm.get("delta_prop_sec", -1))
if abs(dp-13020.0)>1.0:
    raise SystemExit("AE delta_prop expected ~13020, got %s" % dp)
'
echo "  OK  AC-004 AE delta_prop"
pass=$((pass + 1))

echo "==> [11] AC-004 time_model_invalid (W>H)"
run_expect_exit "time_model_invalid exit" 2 \
  "$EXE" validate --input "$ROOT/samples/time_model_invalid.json"

echo "==> [12] AC-004 obsolete duration_sec rejected"
tmp="$(mktemp)"
python3 - <<'PY' >"$tmp"
import json
print(json.dumps({
  "task": {"scenario":"remote_sensing_access","start_time_utc":"2026-12-30T00:00:00Z",
           "duration_sec":172800,"step_sec":10},
  "spacecraft":{"sat_id":"s","epoch_utc":"30 Dec 2026 00:00:00.000","state_type":"keplerian",
    "elements":{"sma_km":6716.14,"ecc":0,"inc_deg":96.8,"raan_deg":76.5,"aop_deg":0,"ta_deg":0}},
  "target":{"type":"ground_point","lon_deg":0,"lat_deg":0,"alt_km":0},
  "sensor":{"type":"optical_linescan","mode":"side_roll_only"},
  "constraints":{"roll_max_deg":30,"require_sunlit":True}
}))
PY
run_expect_exit "obsolete duration_sec exit" 2 \
  "$EXE" validate --input "$tmp"
rm -f "$tmp"

echo "==> [13] AC-004 harness (±W/2 / parse / W<=H)"
HARNESS="$BUILD/ac004-harness"
if [[ ! -x "$HARNESS" ]]; then
  echo "missing harness: $HARNESS (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS"
  got=$?
  set -e
  assert_eq "ac004-harness exit" "$got" "0"
fi

echo "==> [14] AC-007 default cone (omit cone_angle_deg -> 80/10)"
set +e
out="$("$EXE" validate --input "$ROOT/samples/downlink_window.json" --output json 2>/dev/null)"
got=$?
set -e
assert_eq "AC-007 DL validate exit" "$got" "0"
printf '%s' "$out" | python3 -c '
import json,sys
d=json.load(sys.stdin)
dl=(d.get("details") or {}).get("downlink") or {}
cone=float(dl.get("cone_angle_deg", -1))
mine=float(dl.get("min_elevation_deg", -1))
if abs(cone-80.0)>1e-9:
    raise SystemExit("expected cone 80, got %s" % cone)
if abs(mine-10.0)>1e-9:
    raise SystemExit("expected min_elev 10, got %s" % mine)
'
echo "  OK  AC-007 default cone 80 / min_elev 10"
pass=$((pass + 1))

echo "==> [15] AC-007 harness (MaxElev parse / D7 / cone defaults)"
HARNESS7="$BUILD/ac007-harness"
if [[ ! -x "$HARNESS7" ]]; then
  echo "missing harness: $HARNESS7 (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS7"
  got=$?
  set -e
  assert_eq "ac007-harness exit" "$got" "0"
fi

rm -rf "$WORK_BASE"

echo "==> summary: pass=$pass fail=$fail"
if [[ "$fail" -ne 0 ]]; then
  exit 1
fi
echo "access-computer contract test passed"
