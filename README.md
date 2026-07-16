# access-computer

> **产品需求 SSOT**：在 `satellite-workspace` 父仓阅读 [`docs/prd/access-computer-v1.md`](../../docs/prd/access-computer-v1.md)（三场景 I/O、传感器矩阵、Mission 映射、WindowSet 集成验收）。**v0.1 差距矩阵**：[`docs/prd/access-computer-gap-matrix-v0.1.md`](../../docs/prd/access-computer-gap-matrix-v0.1.md)（AC-002）。**接口契约（AC-003）**：[`docs/architecture/ac-003-interface-contract.md`](../../docs/architecture/ac-003-interface-contract.md)（CLI / 退出码 / work_dir / 样本路径）。**样本索引**：[`samples/README.md`](samples/README.md)。本文档为实现与构建说明。

GMAT-based **Access windows compute** CLI plugin (`access.remote_sensing_access`). Distributed mission orchestration lives in [`task-manager`](https://github.com/HotDryNoodle/task-manager); this plugin only computes optical access, attitude, and downlink windows.

> **Migration**: Renamed from `mission-planer` (v0.1.0) to clarify semantics. The old repository is deprecated.

This repository contains three GMAT-backed compute scenarios. Remote sensing follows a two-stage chain: RSA predicts long-horizon candidate windows, then AE refines the selected short window into an execution epoch and attitude.

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
| `remote_sensing_access` | 2 days (`172800 s`) | `10 s` | optical implemented; SAR planned (AC-010) | long-horizon candidate windows and coarse `t0_utc` seed |
| `attitude_estimation` | 5-30 min (`300-1800 s`) | `1 s` | optical implemented; SAR planned (AC-024) | precise execution `t0_utc` and attitude |
| `downlink_window` | 1-2 h (`3600-7200 s`) | `5 s` | `downlink_cone` (optional) | `[start_utc, end_utc, max_elevation_deg]` |

### Sensor support matrix (v0.1.0)

| Sensor | Access windows | Attitude | Notes |
|--------|----------------|----------|-------|
| `optical_linescan` + `side_roll_only` | supported | supported | Off-nadir cone geometry (intent-aligned with GMAT `Ex_OpticalSSO_Access`; no in-repo example script). RSA `phi_deg` = unsigned off-nadir **proxy** (≠ roll). See [`ac-009-rsa-algorithm.md`](../../docs/architecture/ac-009-rsa-algorithm.md). |
| `optical_area_array` + `stare` | supported | supported | AE `pitch_deg` / signed roll via AC-008; RSA window `phi` still off-nadir proxy |
| `sar` | not implemented (AC-010) | not implemented (AC-024) | RSA will use incidence-angle range + look side + LOS + mechanical roll and a coarse min-`|range_rate|` seed. AE will solve zero Doppler at 1 ms and output a full executable attitude. `experimental.allow_sar=true` does not make the current optical path SAR-capable. |
| `downlink_cone` | n/a | n/a | Downlink only; `cone_angle_deg` default `80` → min elev `10°` |

**Algorithm SSOT**：[`docs/architecture/ac-009-rsa-algorithm.md`](../../docs/architecture/ac-009-rsa-algorithm.md)（光学/SAR 的 RSA 长期规划 + AE 临期精化，以及两类传感器的几何差异）。

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
