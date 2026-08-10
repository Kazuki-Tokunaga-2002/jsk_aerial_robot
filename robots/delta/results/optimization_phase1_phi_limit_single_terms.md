# phase1_phi_limit_single_terms 最適化比較結果

条件: `type:=quad`, `version:=module_20260809`, `use_fc_for_att_control=false`, `linear_mode=false`, `nlopt_phi_limit=pi/3`。
軌道: hover 8 s -> circle 半径 0.4 m / 周期 24 s / yaw あり -> post-hover 5 s -> land。

## Variant

| rank | name | score | weights | bag |
| ---: | --- | ---: | --- | --- |
| 1 | `p1_baseline_phi60` | 15.538 | lambda_weight=1.0 | `/tmp/delta_opt_eval_20260810/phase1_phi_limit_single_terms/p1_baseline_phi60/attempt_1/bags/p1_baseline_phi60.bag` |
| 2 | `p1_phi_nominal_10` | 15.716 | lambda_weight=1.0, phi_nominal_weight=10.0 | `/tmp/delta_opt_eval_20260810/phase1_phi_limit_single_terms/p1_phi_nominal_10/attempt_1/bags/p1_phi_nominal_10.bag` |
| 3 | `p1_delta_phi_10` | 15.784 | lambda_weight=1.0, delta_phi_weight=10.0 | `/tmp/delta_opt_eval_20260810/phase1_phi_limit_single_terms/p1_delta_phi_10/attempt_1/bags/p1_delta_phi_10.bag` |
| 4 | `p1_delta_lambda_0p1` | 21.102 | lambda_weight=1.0, delta_lambda_weight=0.1 | `/tmp/delta_opt_eval_20260810/phase1_phi_limit_single_terms/p1_delta_lambda_0p1/attempt_1/bags/p1_delta_lambda_0p1.bag` |

score は比較用の簡易指標で、circle/post-hover の XY error、yaw error、gimbal 角、gimbal step、NLOpt success/iteration を小さいほど良い向きに重み付けしたもの。絶対値そのものより順位を見る。

## Circle 中の主要値

| name | XY RMS [m] | yaw RMS [rad] | \|phi\| RMS [rad] | max step [rad] | max thrust [N] | NLOpt success | iter mean |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `p1_baseline_phi60` | 0.0062 | 0.0114 | 0.0290 | 0.0205 | 9.2588 | 100.0% | 8.14 |
| `p1_phi_nominal_10` | 0.0061 | 0.0117 | 0.0289 | 0.0209 | 9.2782 | 100.0% | 8.12 |
| `p1_delta_phi_10` | 0.0062 | 0.0120 | 0.0291 | 0.0188 | 9.2382 | 100.0% | 10.55 |
| `p1_delta_lambda_0p1` | 0.0104 | 0.0151 | 0.0299 | 0.0215 | 9.2667 | 100.0% | 10.27 |

## 全 phase tracking

| name | hover XY | hover yaw | circle XY | circle yaw | post XY | post yaw |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `p1_baseline_phi60` | 0.0370 | 0.0015 | 0.0062 | 0.0114 | 0.0092 | 0.0391 |
| `p1_phi_nominal_10` | 0.0339 | 0.0021 | 0.0061 | 0.0117 | 0.0095 | 0.0419 |
| `p1_delta_phi_10` | 0.0375 | 0.0025 | 0.0062 | 0.0120 | 0.0087 | 0.0389 |
| `p1_delta_lambda_0p1` | 0.0365 | 0.0018 | 0.0104 | 0.0151 | 0.0107 | 0.0437 |

## メモ

- `phi` の hard constraint は全案で `[-pi/3, pi/3]`。
- NLOpt success が 100% でない案は、制御性能が良く見えても安全側では低評価にする。
- bag は `/tmp` 配下なので、必要なら別途保存する。

## Phase 1 の読み取り

- `p1_baseline_phi60` が総合 1 位。つまり、今回の軌道では **phi bounds を `[-pi/3, pi/3]` にするだけでも十分に良い**。
- `p1_phi_nominal_10` は circle XY が少し良いが、post-hover yaw が悪化して総合では baseline に届かない。
- `p1_delta_phi_10` は circle の最大 gimbal step を `0.0205 -> 0.0188 rad` に下げたが、NLOpt iteration が増え、yaw も少し悪くなった。
- `p1_delta_lambda_0p1` は circle XY/yaw が明確に悪化したので、この方向は少なくとも重み `0.1` では良くない。

## Phase 2 への方針

強い smoothing/nominal penalty は効きすぎる可能性がある。次は以下を試す。

- baseline を再試行して simulation noise を見る。
- `delta_phi` を `10` から `1` または `3` に下げる。
- `phi_nominal` を `10` から `1` に下げる。
- 小さい `delta_phi` と小さい `phi_nominal` の組み合わせを見る。
