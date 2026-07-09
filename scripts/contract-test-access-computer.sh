#!/usr/bin/env bash
# Contract test for access-computer plugin (manifest, validate, dry-run).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
EXE="$BUILD/access-computer"
MANIFEST="$ROOT/configs/plugins/access-computer.json"
SAMPLE="$ROOT/samples/remote_sensing_access.json"
WORK="/tmp/mp_contract_access_computer_$$"

if [[ ! -x "$EXE" ]]; then
  echo "missing executable: $EXE" >&2
  exit 1
fi

echo "==> manifest matches on-disk"
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
PY

echo "==> validate sample"
"$EXE" validate --input "$SAMPLE" >/dev/null

echo "==> dry-run"
mkdir -p "$WORK"
"$EXE" run --input "$SAMPLE" --work-dir "$WORK" --dry-run --output json >/dev/null

rm -rf "$WORK"
echo "access-computer contract test passed"
