#!/usr/bin/env bash
# Contract test for access-computer plugin (AC-003).
# Covers: manifest 1.1, optical/SAR/DL validate+dry-run, negatives, exit codes.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export ACCESS_COMPUTER_DEV_SOURCE_ROOT="$ROOT"
BUILD="${BUILD:-$ROOT/build}"
EXE="$BUILD/access-computer"
MANIFEST="$ROOT/configs/plugins/access-computer.json"
WORK_BASE="/tmp/mp_contract_access_computer_$$"

cleanup() {
  rm -rf "$WORK_BASE"
  rm -f "/tmp/ac_contract_out_$$" "/tmp/ac_contract_err_$$"
}
trap cleanup EXIT

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
  if [[ "$got" != "$want_exit" ]]; then
    echo "  diagnostic stdout for $name:" >&2
    sed -n '1,200p' /tmp/ac_contract_out_$$ >&2
    echo "  diagnostic stderr for $name:" >&2
    sed -n '1,200p' /tmp/ac_contract_err_$$ >&2
  fi
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

validate_ok "SAR-RSA" "sar_rsa.json"
dry_run_ok "SAR-RSA" "sar_rsa.json"

validate_ok "SAR-AE" "sar_attitude_estimation.json"
dry_run_ok "SAR-AE" "sar_attitude_estimation.json"

echo "==> [8] input/output JSON Schema positive and negative contracts"
python3 - "$ROOT" <<'PY'
import copy
import json
import pathlib
import sys

import jsonschema

root = pathlib.Path(sys.argv[1])
input_schema = json.loads(
    (root / "schemas/remote_sensing_access.input.schema.json").read_text()
)
output_schema = json.loads(
    (root / "schemas/remote_sensing_access.output.schema.json").read_text()
)
input_validator = jsonschema.Draft202012Validator(input_schema)
output_validator = jsonschema.Draft202012Validator(output_schema)

sar_rsa = json.loads((root / "samples/sar_rsa.json").read_text())
sar_ae = json.loads((root / "samples/sar_attitude_estimation.json").read_text())
input_validator.validate(sar_rsa)
input_validator.validate(sar_ae)

input_negatives = []
case = copy.deepcopy(sar_rsa)
case.pop("sensor")
input_negatives.append(("missing sensor", case))
case = copy.deepcopy(sar_rsa)
case["sensor"]["mode"] = "stare"
input_negatives.append(("illegal SAR mode", case))
case = copy.deepcopy(sar_rsa)
case["sensor"].pop("azimuth_beamwidth_deg")
input_negatives.append(("missing beamwidth", case))
case = copy.deepcopy(sar_rsa)
case["sensor"]["azimuth_beamwidth_deg"] = 180.0
input_negatives.append(("beamwidth upper bound", case))
case = copy.deepcopy(sar_rsa)
case["constraints"]["roll_max_deg"] = 0.0
input_negatives.append(("roll strict lower bound", case))
case = copy.deepcopy(sar_ae)
case["constraints"]["max_abs_range_rate_mps"] = "0.1"
input_negatives.append(("non-numeric residual", case))
case = copy.deepcopy(sar_ae)
case["selected_window"]["unexpected"] = True
input_negatives.append(("selected_window unknown field", case))
for name, document in input_negatives:
    if not list(input_validator.iter_errors(document)):
        raise SystemExit(f"input schema negative unexpectedly accepted: {name}")

geometry = {
    "incidence_angle_deg": 30.0,
    "look_side": "left",
    "side_look_angle_deg": 28.0,
    "squint_deg": 0.01,
    "roll_deg": 29.0,
    "pitch_deg": 0.1,
    "yaw_deg": 0.2,
    "slant_range_km": 500.0,
    "range_rate_mps": 0.01,
    "doppler_centroid_hz": -0.36,
    "los_clear": True,
}
window = {
    "start_utc": "2026-12-30T00:00:00.000Z",
    "end_utc": "2026-12-30T00:00:02.000Z",
    "duration_sec": 2.0,
    "t0_utc": "2026-12-30T00:00:01.000Z",
    "phi_deg": 29.0,
    "node_id": "local",
    "sar_geometry": geometry,
}
attitude = {
    "mode": "stripmap",
    "attitude_status": "computed",
    "t0_utc": "2026-12-30T00:00:01.000Z",
    "reference_frame": "EarthMJ2000Eq",
    "quaternion_body_to_reference": {
        "order": "wxyz",
        "values": [1.0, 0.0, 0.0, 0.0],
    },
    "roll_deg": 29.0,
    "pitch_deg": 0.1,
    "yaw_deg": 0.2,
    "squint_deg": 0.01,
    "incidence_angle_deg": 30.0,
    "look_side": "left",
    "side_look_angle_deg": 28.0,
    "slant_range_km": 500.0,
    "range_rate_mps": 0.01,
    "doppler_centroid_hz": -0.36,
}
succeeded = {
    "task_id": "task",
    "scenario": "attitude_estimation",
    "status": "succeeded",
    "windows": [window],
    "summary": {"window_count": 1, "duration_total_sec": 2.0},
    "attitude": attitude,
    "artifacts": {"sar_state_path": "/tmp/state.txt"},
    "warnings": [],
}
no_result = {
    "task_id": "task",
    "scenario": "attitude_estimation",
    "status": "no_result",
    "windows": [],
    "summary": {"window_count": 0, "duration_total_sec": 0.0},
    "artifacts": {"sar_state_path": "/tmp/state.txt"},
    "warnings": ["no feasible SAR attitude solution"],
}
minimal_window = {
    "start_utc": "2026-12-30T00:00:00.000Z",
    "end_utc": "2026-12-30T00:00:02.000Z",
    "node_id": "local",
}
optical_ae = copy.deepcopy(succeeded)
optical_ae["windows"] = [minimal_window]
optical_ae["attitude"] = {
    "mode": "side_roll_only",
    "t0_utc": "2026-12-30T00:00:01.000Z",
}
rsa = copy.deepcopy(succeeded)
rsa["scenario"] = "remote_sensing_access"
rsa["windows"] = [minimal_window]
rsa.pop("attitude")
downlink = copy.deepcopy(rsa)
downlink["scenario"] = "downlink_window"
output_validator.validate(succeeded)
output_validator.validate(no_result)
output_validator.validate(optical_ae)
output_validator.validate(rsa)
output_validator.validate(downlink)

output_negatives = []
case = copy.deepcopy(succeeded)
case.pop("attitude")
output_negatives.append(("succeeded AE missing attitude", case))
case = copy.deepcopy(succeeded)
case["attitude"] = {}
output_negatives.append(("succeeded AE empty attitude", case))
case = copy.deepcopy(succeeded)
case["attitude"].pop("quaternion_body_to_reference")
output_negatives.append(("SAR full attitude missing quaternion", case))
for field in (
    "start_utc",
    "end_utc",
    "duration_sec",
    "t0_utc",
    "phi_deg",
    "node_id",
    "sar_geometry",
):
    case = copy.deepcopy(succeeded)
    case["windows"][0].pop(field)
    output_negatives.append((f"SAR AE window missing {field}", case))
case = copy.deepcopy(no_result)
case["windows"] = [window]
output_negatives.append(("no_result with window", case))
case = copy.deepcopy(no_result)
case["summary"]["window_count"] = 1
output_negatives.append(("no_result nonzero summary", case))
case = copy.deepcopy(succeeded)
case["scenario"] = "remote_sensing_access"
output_negatives.append(("RSA output with attitude", case))
for name, document in output_negatives:
    if not list(output_validator.iter_errors(document)):
        raise SystemExit(f"output schema negative unexpectedly accepted: {name}")

print("  OK  JSON Schema positive/negative matrix")
PY
pass=$((pass + 1))

echo "==> [8b] permanent invalid SAR inputs return stable exit 2"
MUTATIONS="$WORK_BASE/mutations"
mkdir -p "$MUTATIONS"
python3 - "$ROOT" "$MUTATIONS" <<'PY'
import copy
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
out = pathlib.Path(sys.argv[2])
rsa = json.loads((root / "samples/sar_rsa.json").read_text())
ae = json.loads((root / "samples/sar_attitude_estimation.json").read_text())
cases = {}
case = copy.deepcopy(rsa); case.pop("sensor"); cases["missing_sensor"] = case
case = copy.deepcopy(rsa); case["sensor"]["mode"] = "stare"; cases["bad_mode"] = case
case = copy.deepcopy(rsa); case["sensor"].pop("azimuth_beamwidth_deg"); cases["missing_beam"] = case
case = copy.deepcopy(rsa); case["sensor"]["azimuth_beamwidth_deg"] = 0.0; cases["bad_beam"] = case
case = copy.deepcopy(rsa); case["constraints"]["max_abs_squint_deg"] = 5.001; cases["bad_squint"] = case
case = copy.deepcopy(rsa); case["constraints"]["roll_max_deg"] = 0.0; cases["bad_roll"] = case
case = copy.deepcopy(ae); case["constraints"]["max_abs_range_rate_mps"] = "0.1"; cases["bad_residual"] = case
case = copy.deepcopy(ae); case["selected_window"]["unexpected"] = True; cases["unknown_window_field"] = case
for name, document in cases.items():
    (out / f"{name}.json").write_text(json.dumps(document))
PY
for invalid in missing_sensor bad_mode missing_beam bad_beam bad_squint bad_roll bad_residual unknown_window_field; do
  run_expect_exit "validate $invalid exit" 2 \
    "$EXE" validate --input "$MUTATIONS/$invalid.json"
  run_expect_exit "dry-run $invalid exit" 2 \
    "$EXE" run --input "$MUTATIONS/$invalid.json" \
      --work-dir "$WORK_BASE/invalid-$invalid" --dry-run --output json
done

echo "==> [8c] SAR incomplete contract negative (exit 2)"
run_expect_exit "SAR incomplete validate exit" 2 \
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

echo "==> [16] AC-005 harness (multi/short/empty clip)"
HARNESS5="$BUILD/ac005-harness"
if [[ ! -x "$HARNESS5" ]]; then
  echo "missing harness: $HARNESS5 (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS5"
  got=$?
  set -e
  assert_eq "ac005-harness exit" "$got" "0"
fi

echo "==> [17] AC-006 harness (illumination flags H1–H8 / V1–V2)"
HARNESS6="$BUILD/ac006-harness"
if [[ ! -x "$HARNESS6" ]]; then
  echo "missing harness: $HARNESS6 (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS6"
  got=$?
  set -e
  assert_eq "ac006-harness exit" "$got" "0"
fi

echo "==> [18] AC-008 harness (pitch/t0 refine G/I/P/A)"
HARNESS8="$BUILD/ac008-harness"
if [[ ! -x "$HARNESS8" ]]; then
  echo "missing harness: $HARNESS8 (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS8"
  got=$?
  set -e
  assert_eq "ac008-harness exit" "$got" "0"
fi

echo "==> [19] AC-010 harness (SAR RSA geometry/report/windows)"
HARNESS10="$BUILD/ac010-harness"
if [[ ! -x "$HARNESS10" ]]; then
  echo "missing harness: $HARNESS10 (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS10"
  got=$?
  set -e
  assert_eq "ac010-harness exit" "$got" "0"
fi

echo "==> [20] AC-024 harness (SAR zero-Doppler/quaternion/no_result)"
HARNESS24="$BUILD/ac024-harness"
if [[ ! -x "$HARNESS24" ]]; then
  echo "missing harness: $HARNESS24 (meson compile -C build)" >&2
  fail=$((fail + 1))
else
  set +e
  "$HARNESS24"
  got=$?
  set -e
  assert_eq "ac024-harness exit" "$got" "0"
fi

echo "==> [21] staged-install relocation loads templates without source tree"
INSTALL_STAGE="$WORK_BASE/install-stage"
RELOCATED="$WORK_BASE/relocated"
meson install -C "$BUILD" --destdir "$INSTALL_STAGE" >/dev/null
INSTALLED_EXE="$(find "$INSTALL_STAGE" -type f -path '*/bin/access-computer' -print -quit)"
if [[ -z "$INSTALLED_EXE" ]]; then
  echo "  FAIL staged install missing access-computer" >&2
  fail=$((fail + 1))
else
  INSTALLED_PREFIX="$(dirname "$(dirname "$INSTALLED_EXE")")"
  mkdir -p "$RELOCATED"
  cp -a "$INSTALLED_PREFIX/." "$RELOCATED/"
  python3 - "$ROOT" "$WORK_BASE/relocated-request.json" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
output = pathlib.Path(sys.argv[2])
request = json.loads((root / "samples/sar_rsa.json").read_text())
request["gmat"] = {"install_root": "/usr", "console_binary": "/bin/true"}
output.write_text(json.dumps(request))
PY
  set +e
  env -u ACCESS_COMPUTER_DEV_SOURCE_ROOT -u ACCESS_COMPUTER_DATA_DIR \
    "$RELOCATED/bin/access-computer" run \
      --input "$WORK_BASE/relocated-request.json" \
      --work-dir "$WORK_BASE/relocated-work" --output json \
      >"$WORK_BASE/relocated.out" 2>"$WORK_BASE/relocated.err"
  got=$?
  set -e
  assert_eq "relocated fake-GMAT exit" "$got" "5"
  if [[ -s "$WORK_BASE/relocated-work/rendered.script" ]] &&
     grep -q "TargetA.HorizonReference = Ellipsoid" \
       "$WORK_BASE/relocated-work/rendered.script" &&
     ! grep -q "template unavailable" "$WORK_BASE/relocated.out" \
       "$WORK_BASE/relocated.err"; then
    echo "  OK  relocated installed SAR template loaded"
    pass=$((pass + 1))
  else
    echo "  FAIL relocated installed SAR template was not loaded" >&2
    fail=$((fail + 1))
  fi
fi

if [[ "${RUN_GMAT_INTEGRATION:-0}" == "1" ]]; then
  echo "==> [22] real GMAT SAR RSA -> full-window AE -> independent replay"
  REAL="$WORK_BASE/real-gmat"
  mkdir -p "$REAL"
  python3 - "$ROOT" "$REAL/rsa-request.json" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
output = pathlib.Path(sys.argv[2])
request = json.loads((root / "samples/sar_rsa.json").read_text())
request["task"]["compute_horizon_sec"] = 14400.0
output.write_text(json.dumps(request))
PY
  set +e
  "$EXE" run --input "$REAL/rsa-request.json" \
    --work-dir "$REAL/rsa" --output json-pretty >"$REAL/rsa-result.json"
  rsa_exit=$?
  set -e
  assert_eq "real SAR RSA exit" "$rsa_exit" "0"
  if [[ "$rsa_exit" == "0" ]]; then
    python3 - "$ROOT" "$REAL/rsa-result.json" "$REAL/ae-request.json" <<'PY'
import datetime as dt
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
rsa_result = json.loads(pathlib.Path(sys.argv[2]).read_text())
output = pathlib.Path(sys.argv[3])
if rsa_result.get("status") != "succeeded" or not rsa_result.get("windows"):
    raise SystemExit("real SAR RSA did not return a window")
request = json.loads((root / "samples/sar_attitude_estimation.json").read_text())
selected = rsa_result["windows"][0]
request["selected_window"] = selected
parse = lambda value: dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
format_utc = lambda value: value.isoformat(timespec="milliseconds").replace("+00:00", "Z")
start = parse(selected["start_utc"])
end = parse(selected["end_utc"])
task_start = start - dt.timedelta(seconds=60)
request["task"]["start_time_utc"] = format_utc(task_start)
request["task"]["compute_horizon_sec"] = max(
    300.0, (end - start).total_seconds() + 120.0
)
output.write_text(json.dumps(request))
PY
    run_expect_exit "full AC-010 window validates for AE" 0 \
      "$EXE" validate --input "$REAL/ae-request.json"
    set +e
    "$EXE" run --input "$REAL/ae-request.json" \
      --work-dir "$REAL/ae" --output json-pretty >"$REAL/ae-result.json"
    ae_exit=$?
    set -e
    assert_eq "real SAR AE exit" "$ae_exit" "0"
    if [[ "$ae_exit" == "0" ]]; then
      run_expect_exit "real RSA independent replay" 0 \
        "$HARNESS10" "$REAL/rsa/sar_state_j2000.txt" \
          "$REAL/rsa-result.json"
      run_expect_exit "real AE independent replay" 0 \
        "$HARNESS24" "$REAL/ae/sar_state_j2000.txt" \
          "$REAL/ae-result.json" "$REAL/ae-request.json"
      python3 - "$REAL/ae-result.json" <<'PY'
import json
import sys

result = json.load(open(sys.argv[1]))
attitude = result["attitude"]
window = result["windows"][0]
if not attitude["t0_utc"].endswith("Z") or "." not in attitude["t0_utc"]:
    raise SystemExit("AE t0 is not canonical millisecond UTC")
if abs(attitude["squint_deg"]) > 5.0:
    raise SystemExit("AE actual squint exceeds effective beam gate")
if abs(attitude["roll_deg"]) > 70.0:
    raise SystemExit("AE mechanical roll exceeds gate")
if not (window["start_utc"] <= attitude["t0_utc"] <= window["end_utc"]):
    raise SystemExit("AE refined window does not contain t0")
print("  OK  real AE millisecond/squint/roll/refined-window invariants")
PY
      pass=$((pass + 1))
    fi
  fi

  echo "==> [22b] real GMAT non-integer horizon endpoint regression"
  python3 - "$ROOT" "$REAL/tail-request.json" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
output = pathlib.Path(sys.argv[2])
request = json.loads((root / "samples/sar_rsa.json").read_text())
request["task"]["compute_horizon_sec"] = 60.5
request["task"]["working_time_sec"] = 60.0
request["task"]["step_sec"] = 10.0
output.write_text(json.dumps(request))
PY
  set +e
  "$EXE" run --input "$REAL/tail-request.json" \
    --work-dir "$REAL/tail" --output json-pretty >"$REAL/tail-result.json"
  tail_exit=$?
  set -e
  if [[ "$tail_exit" == "0" || "$tail_exit" == "4" ]]; then
    echo "  OK  real tail-horizon planner exit ($tail_exit)"
    pass=$((pass + 1))
  else
    echo "  FAIL real tail-horizon planner exit: got=$tail_exit want=0|4" >&2
    fail=$((fail + 1))
  fi
  python3 - "$REAL/tail/sar_state_j2000.txt" <<'PY'
import datetime as dt
import pathlib
import sys

lines = pathlib.Path(sys.argv[1]).read_text().splitlines()
if len(lines) != 8:
    raise SystemExit(f"expected start + 6 full steps + terminal = 8 rows, got {len(lines)}")
tokens = lines[-1].split()
last = dt.datetime.strptime(" ".join(tokens[:4]), "%d %b %Y %H:%M:%S.%f")
want = dt.datetime(2026, 12, 30, 0, 1, 0, 500000)
if last != want:
    raise SystemExit(f"tail report endpoint mismatch: {last} != {want}")
print("  OK  real GMAT report reaches exact non-integer horizon endpoint")
PY
  pass=$((pass + 1))
fi

echo "==> summary: pass=$pass fail=$fail"
if [[ "$fail" -ne 0 ]]; then
  exit 1
fi
echo "access-computer contract test passed"
