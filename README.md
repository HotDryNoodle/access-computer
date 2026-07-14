# access-computer

> **产品需求 SSOT**：在 `satellite-workspace` 父仓阅读 [`docs/prd/access-computer-v1.md`](../../docs/prd/access-computer-v1.md)（三场景 I/O、传感器矩阵、Mission 映射、WindowSet 集成验收）。**v0.1 差距矩阵**：[`docs/prd/access-computer-gap-matrix-v0.1.md`](../../docs/prd/access-computer-gap-matrix-v0.1.md)（AC-002）。**接口契约（AC-003）**：[`docs/architecture/ac-003-interface-contract.md`](../../docs/architecture/ac-003-interface-contract.md)（CLI / 退出码 / work_dir / 样本路径）。**样本索引**：[`samples/README.md`](samples/README.md)。本文档为实现与构建说明。

GMAT-based **Access windows compute** CLI plugin (`access.remote_sensing_access`). Distributed mission orchestration lives in [`task-manager`](https://github.com/HotDryNoodle/task-manager); this plugin only computes optical access, attitude, and downlink windows.

> **Migration**: Renamed from `mission-planer` (v0.1.0) to clarify semantics. The old repository is deprecated.

This repository contains **access-window compute logic only** — three GMAT-backed compute scenarios.

## Build

```bash
meson subprojects download
meson setup build
meson compile -C build
```

Depends on **satellite-plugin-sdk** via Meson wrap (`subprojects/satellite-plugin-sdk.wrap`).

## Contract test

```bash
./scripts/contract-test-access-computer.sh
```

## Business scenarios

| Scenario | Horizon | Step | Sensor | Key outputs |
|----------|---------|------|--------|-------------|
| `remote_sensing_access` | 2 days (`172800 s`) | `10 s` | optical linescan / area array | access windows, `t0_utc`, `phi_deg` |
| `attitude_estimation` | 5-30 min (`300-1800 s`) | `1 s` | optical area array (`stare`) | `attitude.t0_utc`, `attitude.phi_deg` |
| `downlink_window` | 1-2 h (`3600-7200 s`) | `5 s` | `downlink_cone` (optional) | `[start_utc, end_utc]` contact windows |

### Sensor support matrix (v0.1.0)

| Sensor | Access windows | Attitude | Notes |
|--------|----------------|----------|-------|
| `optical_linescan` + `side_roll_only` | supported | supported | Matches GMAT `Ex_OpticalSSO_Access` geometry |
| `optical_area_array` + `stare` | supported | supported | `pitch_deg` is placeholder only |
| `sar` | not implemented | not implemented | rejected unless `experimental.allow_sar=true` |
| `downlink_cone` | n/a | n/a | Downlink only; `cone_angle_deg` default `65` |

## CLI usage

```bash
./build/access-computer manifest --output json
./build/access-computer validate --input samples/remote_sensing_access.json
./build/access-computer run --input samples/remote_sensing_access_gmat_compatible_2day.json --work-dir /tmp/mp_gmat_2day --output json
```

Environment: `GMAT_ROOT` — default GMAT install root if not set in request JSON.

## GMAT optical regression

Set `RUN_GMAT_INTEGRATION=1` with `./scripts/build_and_smoke.sh` for full GMAT regression.

## Exit codes

| Code | Meaning |
|------|---------|
| 0 | success |
| 2 | validation error |
| 3 | missing dependency |
| 4 | no business result |
| 5 | retryable failure |
| 6 | fatal failure（预留；当前路径不返回） |
| 64 | CLI usage error |

完整触发矩阵见 [AC-003 接口契约](../../docs/architecture/ac-003-interface-contract.md) §3。
