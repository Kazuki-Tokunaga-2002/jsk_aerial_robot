# phase3_phi_nominal_tuning 最適化比較結果

条件: `type:=quad`, `version:=module_20260809`, `use_fc_for_att_control=false`, `linear_mode=false`, `nlopt_phi_limit=pi/3`。
軌道: hover 8 s -> circle 半径 0.4 m / 周期 24 s / yaw あり -> post-hover 5 s -> land。

## Variant

| rank | name | score | weights | bag |
| ---: | --- | ---: | --- | --- |
| 1 | `p3_phi_nominal_1_balance_0p05` | 15.445 | lambda_weight=1.0, phi_nominal_weight=1.0, lambda_balance_weight=0.05 | `/tmp/delta_opt_eval_20260810/phase3_phi_nominal_tuning/p3_phi_nominal_1_balance_0p05/attempt_1/bags/p3_phi_nominal_1_balance_0p05.bag` |
| 2 | `p3_phi_nominal_1_repeat` | 15.501 | lambda_weight=1.0, phi_nominal_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase3_phi_nominal_tuning/p3_phi_nominal_1_repeat/attempt_1/bags/p3_phi_nominal_1_repeat.bag` |
| 3 | `p3_phi_nominal_3` | 15.507 | lambda_weight=1.0, phi_nominal_weight=3.0 | `/tmp/delta_opt_eval_20260810/phase3_phi_nominal_tuning/p3_phi_nominal_3/attempt_1/bags/p3_phi_nominal_3.bag` |
| 4 | `p3_phi_nominal_0p3` | 15.697 | lambda_weight=1.0, phi_nominal_weight=0.3 | `/tmp/delta_opt_eval_20260810/phase3_phi_nominal_tuning/p3_phi_nominal_0p3/attempt_1/bags/p3_phi_nominal_0p3.bag` |
| 5 | `p3_phi_nominal_1_dphi_0p3` | 16.457 | lambda_weight=1.0, delta_phi_weight=0.3, phi_nominal_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase3_phi_nominal_tuning/p3_phi_nominal_1_dphi_0p3/attempt_1/bags/p3_phi_nominal_1_dphi_0p3.bag` |

score は比較用の簡易指標で、circle/post-hover の XY error、yaw error、gimbal 角、gimbal step、NLOpt success/iteration を小さいほど良い向きに重み付けしたもの。絶対値そのものより順位を見る。

## Circle 中の主要値

| name | XY RMS [m] | yaw RMS [rad] | \|phi\| RMS [rad] | max step [rad] | max thrust [N] | NLOpt success | iter mean |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `p3_phi_nominal_1_balance_0p05` | 0.0061 | 0.0122 | 0.0283 | 0.0214 | 9.2286 | 100.0% | 8.09 |
| `p3_phi_nominal_1_repeat` | 0.0063 | 0.0117 | 0.0283 | 0.0191 | 9.2192 | 100.0% | 8.14 |
| `p3_phi_nominal_3` | 0.0062 | 0.0116 | 0.0279 | 0.0204 | 9.2266 | 100.0% | 8.14 |
| `p3_phi_nominal_0p3` | 0.0061 | 0.0115 | 0.0341 | 0.0196 | 9.2433 | 100.0% | 8.07 |
| `p3_phi_nominal_1_dphi_0p3` | 0.0070 | 0.0118 | 0.0282 | 0.0206 | 9.2406 | 100.0% | 9.45 |

## 全 phase tracking

| name | hover XY | hover yaw | circle XY | circle yaw | post XY | post yaw |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `p3_phi_nominal_1_balance_0p05` | 0.0347 | 0.0023 | 0.0061 | 0.0122 | 0.0091 | 0.0402 |
| `p3_phi_nominal_1_repeat` | 0.0324 | 0.0024 | 0.0063 | 0.0117 | 0.0090 | 0.0398 |
| `p3_phi_nominal_3` | 0.0351 | 0.0024 | 0.0062 | 0.0116 | 0.0092 | 0.0408 |
| `p3_phi_nominal_0p3` | 0.0325 | 0.0032 | 0.0061 | 0.0115 | 0.0088 | 0.0388 |
| `p3_phi_nominal_1_dphi_0p3` | 0.0046 | 0.0016 | 0.0070 | 0.0118 | 0.0090 | 0.0391 |

## メモ

- `phi` の hard constraint は全案で `[-pi/3, pi/3]`。
- NLOpt success が 100% でない案は、制御性能が良く見えても安全側では低評価にする。
- bag は `/tmp` 配下なので、必要なら別途保存する。

## Phase 3 の読み取り

- Phase 3 内では `p3_phi_nominal_1_balance_0p05` が僅差 1 位。ただし `p3_phi_nominal_1_repeat` との差は小さく、yaw と post-hover は単独の `phi_nominal=1` のほうが少し良い。
- `p3_phi_nominal_3` は `phi_nominal=1` とほぼ同等だが、強くする明確な利点は見えない。
- `p3_phi_nominal_0p3` は gimbal step と post-hover yaw は良いが、circle の `|phi| RMS` が大きい。
- `p3_phi_nominal_1_dphi_0p3` は hover は非常に良いが、circle XY と iteration が悪化した。Phase 1/2 と同じく、`delta_phi` penalty は今回の軌道では主推奨にしない。

## 最終判断

3 フェーズ全体で見ると、最終推奨は **`phi_limit=pi/3`, `phi_nominal_weight=1.0`, `delta_phi_weight=0.0`, `lambda_balance_weight=0.0`**。

理由:

- `phi_nominal_weight=1.0` 単独は Phase 2 で全体最良、Phase 3 再試行でも上位。
- baseline より circle XY と gimbal 角が改善する。
- `delta_phi` penalty を足すと iteration が増えやすく、circle XY が悪化しやすい。
- `lambda_balance=0.05` は僅差で良いが、単発結果なので本採用にはまだ弱い。
