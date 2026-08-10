# Delta nonlinear allocation 最適化 3フェーズ比較まとめ

実施日: 2026-08-10  
対象: `delta`, `type:=quad`, `version:=module_20260809`  
固定条件:

- `use_fc_for_att_control: false`
- `linear_mode: false`
- `nlopt_phi_limit: 1.0471975512` rad, つまり `phi_i in [-pi/3, pi/3]`
- 同一軌道: hover 8 s -> circle 半径 0.4 m / 周期 24 s / yaw あり -> post-hover 5 s -> land

## 最終結論

最終推奨はこれ。

```yaml
controller:
  linear_mode: false
  use_fc_for_att_control: false
  nlopt_phi_limit: 1.0471975512
  nlopt_lambda_weight: 1.0
  nlopt_delta_lambda_weight: 0.0
  nlopt_delta_phi_weight: 0.0
  nlopt_phi_nominal: 0.0
  nlopt_phi_nominal_weight: 1.0
  nlopt_lambda_balance_weight: 0.0
```

`robots/delta/config/quad/module_20260809/DeltaControl.yaml` には上記のうち、最終推奨の `nlopt_phi_nominal_weight: 1.0` まで反映済み。

## 実装した最適化形式

従来:

```text
minimize sum(lambda_i^2)
subject to Q(phi) lambda = target_wrench
           0 <= lambda_i <= lambda_max
           -pi <= phi_i <= pi
```

今回:

```text
minimize
  w_lambda        * sum(lambda_i^2)
  + w_dlambda     * sum((lambda_i - lambda_i_prev)^2)
  + w_dphi        * sum(wrap(phi_i - phi_i_prev)^2)
  + w_phi_nominal * sum(wrap(phi_i - phi_nominal)^2)
  + w_balance     * sum((lambda_i - mean(lambda))^2)

subject to
  Q(phi) lambda = target_wrench
  0 <= lambda_i <= lambda_max
  -phi_limit <= phi_i <= phi_limit
```

今回の最終推奨では、`w_phi_nominal=1.0` のみ追加し、`phi` を 0 rad 付近に弱く寄せる。

## 3フェーズの結果

| phase | best variant | score | 主な読み取り |
| --- | --- | ---: | --- |
| Phase 1 | `p1_baseline_phi60` | 15.538 | `phi` 制約だけで十分良い。強い単独 penalty は大きな改善なし。 |
| Phase 2 | `p2_phi_nominal_1` | 14.588 | `phi_nominal_weight=1.0` が全体最良。circle XY と gimbal 角が改善。 |
| Phase 3 | `p3_phi_nominal_1_balance_0p05` | 15.445 | `lambda_balance=0.05` は僅差で良いが、単発差。安定採用は `phi_nominal=1.0` 単独。 |

全体で同じ重みを複数回見た平均では、`phi_nominal_weight=1.0` 単独が最良。

```text
phi_nominal_weight=1.0 単独: average score 15.044
baseline phi limit only:       average score 15.486
```

## 代表値

### Baseline: `phi_limit=pi/3`, 追加 penalty なし

Phase 2 baseline repeat:

| 指標 | 値 |
| --- | ---: |
| circle XY RMS | 0.0061 m |
| circle yaw RMS | 0.0120 rad |
| circle \|phi\| RMS | 0.0280 rad |
| circle max gimbal step | 0.0227 rad |
| NLOpt success | 100% |
| NLOpt iteration mean | 8.09 |

### Recommended: `phi_nominal_weight=1.0`

Phase 2 best:

| 指標 | 値 |
| --- | ---: |
| circle XY RMS | 0.0055 m |
| circle yaw RMS | 0.0115 rad |
| circle \|phi\| RMS | 0.0263 rad |
| circle max gimbal step | 0.0224 rad |
| NLOpt success | 100% |
| NLOpt iteration mean | 8.42 |

Baseline と比べて、circle XY と gimbal 角が少し改善した。iteration は少し増えるが許容範囲。

## 採用しなかった案

### `delta_phi` penalty

`delta_phi` は gimbal step を減らす狙いだったが、今回の軌道では iteration が増えやすく、circle XY が悪化しやすかった。

- `delta_phi_weight=10`: step は少し減るが iteration 増加。
- `delta_phi_weight=1` または `3`: baseline より総合 score が悪い。
- `phi_nominal=1 + delta_phi=0.3`: hover は良いが circle XY が悪化。

### `delta_lambda` penalty

`delta_lambda_weight=0.1` は明確に悪化した。

- circle XY RMS: `0.0062 -> 0.0104 m`
- circle yaw RMS: `0.0114 -> 0.0151 rad`

今回の軌道では採用しない。

### `lambda_balance` penalty

`phi_nominal=1 + lambda_balance=0.05` は Phase 3 内では僅差 1 位だったが、差が小さく、yaw/post-hover は単独 `phi_nominal=1` のほうが少し良い。  
現時点ではまだ本採用せず、今後の候補として残す。

## 保存ファイル

Phase ごとの詳細:

- `robots/delta/results/optimization_phase1_phi_limit_single_terms.md`
- `robots/delta/results/optimization_phase2_small_weights.md`
- `robots/delta/results/optimization_phase3_phi_nominal_tuning.md`

集計 JSON:

- `robots/delta/results/optimization_phase1_phi_limit_single_terms.json`
- `robots/delta/results/optimization_phase2_small_weights.json`
- `robots/delta/results/optimization_phase3_phi_nominal_tuning.json`

bag は `/tmp/delta_opt_eval_20260810/...` 配下。

## 次にやるなら

今回の結果からは、次は以下の順で見るのが良い。

1. `phi_nominal_weight=1.0` を固定して、半径や周期を変えた circle で再評価する。
2. yaw を強めに振る軌道で、`phi_nominal_weight=1.0` が yaw tracking に悪影響を出さないか見る。
3. `lambda_balance_weight=0.02, 0.05, 0.1` を複数試行で再評価する。
4. `delta_phi` は hard に入れず、必要なら very small weight `0.05` 程度から再検討する。
