# phase2_small_weights 最適化比較結果

条件: `type:=quad`, `version:=module_20260809`, `use_fc_for_att_control=false`, `linear_mode=false`, `nlopt_phi_limit=pi/3`。
軌道: hover 8 s -> circle 半径 0.4 m / 周期 24 s / yaw あり -> post-hover 5 s -> land。

## Variant

| rank | name | score | weights | bag |
| ---: | --- | ---: | --- | --- |
| 1 | `p2_phi_nominal_1` | 14.588 | lambda_weight=1.0, phi_nominal_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase2_small_weights/p2_phi_nominal_1/attempt_2/bags/p2_phi_nominal_1.bag` |
| 2 | `p2_baseline_repeat` | 15.434 | lambda_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase2_small_weights/p2_baseline_repeat/attempt_1/bags/p2_baseline_repeat.bag` |
| 3 | `p2_combo_phi1_nom1` | 15.766 | lambda_weight=1.0, delta_phi_weight=1.0, phi_nominal_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase2_small_weights/p2_combo_phi1_nom1/attempt_1/bags/p2_combo_phi1_nom1.bag` |
| 4 | `p2_delta_phi_1` | 15.957 | lambda_weight=1.0, delta_phi_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase2_small_weights/p2_delta_phi_1/attempt_1/bags/p2_delta_phi_1.bag` |
| 5 | `p2_delta_phi_3` | 16.539 | lambda_weight=1.0, delta_phi_weight=3.0 | `/tmp/delta_opt_eval_20260810/phase2_small_weights/p2_delta_phi_3/attempt_2/bags/p2_delta_phi_3.bag` |

score は比較用の簡易指標で、circle/post-hover の XY error、yaw error、gimbal 角、gimbal step、NLOpt success/iteration を小さいほど良い向きに重み付けしたもの。絶対値そのものより順位を見る。

## Circle 中の主要値

| name | XY RMS [m] | yaw RMS [rad] | \|phi\| RMS [rad] | max step [rad] | max thrust [N] | NLOpt success | iter mean |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `p2_phi_nominal_1` | 0.0055 | 0.0115 | 0.0263 | 0.0224 | 9.2518 | 100.0% | 8.42 |
| `p2_baseline_repeat` | 0.0061 | 0.0120 | 0.0280 | 0.0227 | 9.2049 | 100.0% | 8.09 |
| `p2_combo_phi1_nom1` | 0.0065 | 0.0119 | 0.0238 | 0.0219 | 9.2601 | 100.0% | 9.89 |
| `p2_delta_phi_1` | 0.0063 | 0.0116 | 0.0280 | 0.0252 | 9.2153 | 100.0% | 9.89 |
| `p2_delta_phi_3` | 0.0061 | 0.0117 | 0.0375 | 0.0246 | 9.2461 | 100.0% | 10.15 |

## 全 phase tracking

| name | hover XY | hover yaw | circle XY | circle yaw | post XY | post yaw |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `p2_phi_nominal_1` | 0.0111 | 0.0029 | 0.0055 | 0.0115 | 0.0087 | 0.0392 |
| `p2_baseline_repeat` | 0.0344 | 0.0021 | 0.0061 | 0.0120 | 0.0090 | 0.0398 |
| `p2_combo_phi1_nom1` | 0.0391 | 0.0019 | 0.0065 | 0.0119 | 0.0090 | 0.0399 |
| `p2_delta_phi_1` | 0.0399 | 0.0018 | 0.0063 | 0.0116 | 0.0091 | 0.0397 |
| `p2_delta_phi_3` | 0.0262 | 0.0111 | 0.0061 | 0.0117 | 0.0091 | 0.0390 |

## メモ

- `phi` の hard constraint は全案で `[-pi/3, pi/3]`。
- NLOpt success が 100% でない案は、制御性能が良く見えても安全側では低評価にする。
- bag は `/tmp` 配下なので、必要なら別途保存する。

## Phase 2 の読み取り

- `p2_phi_nominal_1` が総合 1 位。`phi` を 0 rad 付近に弱く寄せるだけで、circle XY RMS が `0.0061 -> 0.0055 m` に改善し、circle の `|phi| RMS` も `0.0280 -> 0.0263 rad` に下がった。
- `p2_combo_phi1_nom1` は `|phi| RMS` は最小だが、circle XY と iteration が悪化した。`delta_phi` を足すと solver が少し重くなりやすい。
- `p2_delta_phi_1` と `p2_delta_phi_3` は単独では baseline より良くない。gimbal smoothing は今回の軌道では主目的にしないほうがよさそう。
- `p2_delta_phi_3` と `p2_phi_nominal_1` は attempt 2 で成功した。失敗の多くは takeoff 待ちの timing/速度低下由来に見えるが、安定性の観点では attempt 1 で通る案を優先したい。

## Phase 3 への方針

`phi_nominal_weight=1` 周辺を最後に細かく見る。

- `phi_nominal_weight` を `0.3`, `1`, `3` で比較する。
- `phi_nominal_weight=1` に弱い `delta_phi=0.3` を足して、gimbal step を少しだけ抑えられるか見る。
- `phi_nominal_weight=1` に `lambda_balance_weight=0.05` を足して、推力配分を均す効果があるか見る。
