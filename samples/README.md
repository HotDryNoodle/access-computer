# access-computer samples

> 契约 SSOT：父仓 [`docs/architecture/ac-003-interface-contract.md`](../../../docs/architecture/ac-003-interface-contract.md)
> 时间模型（AC-004）：[`docs/architecture/ac-004-epoch-alignment.md`](../../../docs/architecture/ac-004-epoch-alignment.md)

| 文件 | 场景 | 角色 | 预期 |
|------|------|------|------|
| `remote_sensing_access.json` | `remote_sensing_access` | 光学主正例；H=172800，W=200 | validate 0；`delta_prop=0`；dry-run OK |
| `remote_sensing_access_gmat_compatible_2day.json` | `remote_sensing_access` | GMAT 2day 回归 | validate 0；真跑需 GMAT |
| `short_access.json` | `remote_sensing_access` | 短窗；H=600，W=200 | validate 0 |
| `attitude_estimation.json` | `attitude_estimation` | 凝视；H=W=900；start=03:37 → Δ_prop≈13020 | validate 0；dry-run OK |
| `sar_rsa.json` | `remote_sensing_access` | AC-010 SAR stripmap 长期候选窗；含必填全方位 beamwidth | validate 0；真跑输出粗 min-`|range_rate|`、actual squint、机械姿态诊断与 `sar_geometry` |
| `sar_attitude_estimation.json` | `attitude_estimation` | AC-024 已选窗零多普勒精化；`selected_window` 可替换为 AC-010 完整窗口 | validate 0；真跑输出绝对 UTC 1 ms `t0`、actual squint、refined window 和完整四元数姿态 |
| `downlink_window.json` | `downlink_window` | 下行默认；H=W=7200；**省略** `cone_angle_deg` → 80°/门限 10° | validate 0；details.downlink 回显 |
| `downlink_window_permissive.json` | `downlink_window` | cone 90° | validate 0 |
| `sar_unsupported.json` | `remote_sensing_access` | SAR 缺 mode/入射角/视向侧/载频/beamwidth 负例 | validate **exit 2** |
| `time_model_invalid.json` | `remote_sensing_access` | W>H 负例（AC-004） | validate **exit 2** |

## 时间字段（AC-004）

| 字段 | 含义 |
|------|------|
| `spacecraft.epoch_utc` | 轨道递推参考历元 |
| `task.start_time_utc` | 任务计算起点（ISO-8601 `Z`） |
| `task.compute_horizon_sec` | 外推计算窗 H：`[start, start+H]` |
| `task.working_time_sec` | 工作窗 W；光学/RSA 按 `[t0±W/2]`；SAR AE 再与 selected/refined geometry window 求交；DL 不裁剪 |
| （派生）`delta_prop_sec` | `start − epoch`，仅 validate `details.time_model` / GMAT 快进 |

已删除：`task.duration_sec`、`task.elapsed_start_sec`、`constraints.task_duration_sec`。

## 命令速查

```bash
EXE=./build/access-computer
"$EXE" validate --input samples/remote_sensing_access.json --output json
"$EXE" validate --input samples/sar_rsa.json --output json
"$EXE" validate --input samples/sar_attitude_estimation.json --output json
"$EXE" validate --input samples/time_model_invalid.json   # expect exit 2
```
