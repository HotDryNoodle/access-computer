#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

meson subprojects download
meson setup "$BUILD" "$@"
meson compile -C "$BUILD"

echo "==> Contract test (access-computer)"
"$ROOT/scripts/contract-test-access-computer.sh"

echo "==> CLI smoke"
"$BUILD/access-computer" manifest --output json >/dev/null
"$BUILD/access-computer" --version
"$BUILD/access-computer" validate --input "$ROOT/samples/remote_sensing_access.json"
"$BUILD/access-computer" run --input "$ROOT/samples/remote_sensing_access.json" --work-dir /tmp/mp_smoke --dry-run

if [[ "${RUN_GMAT_INTEGRATION:-0}" == "1" ]]; then
  echo "==> GMAT integration regression"
  WORK="$ROOT/var/regression/smoke_gmat_compatible_2day"
  mkdir -p "$WORK"
  "$BUILD/access-computer" run --input "$ROOT/samples/remote_sensing_access_gmat_compatible_2day.json" --work-dir "$WORK" --output json >/dev/null
fi

echo "Build and smoke checks completed."
