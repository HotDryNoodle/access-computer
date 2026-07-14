# access-computer samples

> 契约 SSOT：父仓 [`docs/architecture/ac-003-interface-contract.md`](../../../docs/architecture/ac-003-interface-contract.md)

| 文件 | 场景 | 角色 | 预期 |
|------|------|------|------|
| `remote_sensing_access.json` | `remote_sensing_access` | 光学主正例（linescan / side_roll_only） | validate 0；dry-run `status=dry_run` |
| `remote_sensing_access_gmat_compatible_2day.json` | `remote_sensing_access` | GMAT 2day 回归输入 | validate 0；真跑需 GMAT（非 AC-003 contract） |
| `short_access.json` | `remote_sensing_access` | 短窗正例（Golden 候选，AC-013） | validate 0；可缺显式 `propagation_profile`（用默认） |
| `attitude_estimation.json` | `attitude_estimation` | 凝视姿态正例（area_array / stare） | validate 0；dry-run OK |
| `downlink_window.json` | `downlink_window` | 下行正例（cone 65°） | validate 0；dry-run OK |
| `downlink_window_permissive.json` | `downlink_window` | 下行宽锥（90°） | validate 0 |
| `sar_unsupported.json` | `remote_sensing_access` | **SAR 负例**（无 `experimental.allow_sar`） | validate **exit 2** |

## 命令速查

```bash
EXE=./build/access-computer   # 或 install 前缀下的 access-computer

"$EXE" validate --input samples/remote_sensing_access.json
"$EXE" validate --input samples/attitude_estimation.json
"$EXE" validate --input samples/downlink_window.json
"$EXE" validate --input samples/sar_unsupported.json   # expect exit 2

"$EXE" run --input samples/remote_sensing_access.json \
  --work-dir /tmp/ac_rsa --dry-run --output json
"$EXE" run --input samples/attitude_estimation.json \
  --work-dir /tmp/ac_ae --dry-run --output json
"$EXE" run --input samples/downlink_window.json \
  --work-dir /tmp/ac_dl --dry-run --output json
```

## 待新增（非本目录当前交付）

| 文件 | 归属 | 说明 |
|------|------|------|
| `sar_mvp.json` | AC-010 | SAR roll-only 正例；默认通过 validate，不依赖 `experimental.allow_sar` |
