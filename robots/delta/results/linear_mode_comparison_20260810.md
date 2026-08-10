# linear_mode true/false simulation 比較結果

実施日: 2026-08-10  
対象: `delta`, `type:=quad`, `version:=module_20260809`  
比較対象:

- `linear_mode: true`
- `linear_mode: false`

固定条件:

- `use_fc_for_att_control: false`
- Gazebo simulation, `headless:=true`
- 同じ軌道: hover -> circle + yaw -> post-hover -> land
- circle 条件: 半径 `0.4 m`, 周期 `24 s`, yaw 追従あり, 1周

## 結論

今回の 1 回の simulation 結果では、全体としては **`linear_mode: false` のほうが良い**。

特に circle 中の XY tracking error が大きく改善しており、gimbal 角も小さく、角度変化も滑らかだった。  
一方で yaw tracking だけを見ると `linear_mode: true` のほうが少し良い。

したがって、現在の quad/module_20260809 で hover/circle/yaw を飛ばすなら、まずは **`linear_mode: false` を基本候補**にしてよい。yaw 精度を最優先する場合だけ、追加確認したほうがよい。

## 実験データ

bag:

- `linear_mode: true`: `/tmp/delta_alloc_eval_bags/linear_true_20260810_005648.bag`
- `linear_mode: false`: `/tmp/delta_alloc_eval_bags/linear_false_20260810_010002.bag`

集計:

- `/tmp/delta_alloc_eval_bags/summary.json`

注意: bag と summary は `/tmp` 配下なので、必要なら永続保存する。

## 主要結果

### Tracking error

| phase | 指標 | linear_mode=true | linear_mode=false | 評価 |
| --- | ---: | ---: | ---: | --- |
| hover | XY RMS | 0.0029 m | 0.0019 m | false が少し良い |
| hover | Z RMS | 0.0144 m | 0.0148 m | ほぼ同等 |
| hover | yaw RMS | 0.0018 rad | 0.0015 rad | false が少し良い |
| circle | XY RMS | 0.0276 m | 0.0062 m | false が大きく良い |
| circle | Z RMS | 0.0013 m | 0.0013 m | 同等 |
| circle | yaw RMS | 0.0047 rad | 0.0117 rad | true が良い |
| post-hover | XY RMS | 0.0264 m | 0.0094 m | false が良い |
| post-hover | Z RMS | 0.0012 m | 0.0011 m | ほぼ同等 |
| post-hover | yaw RMS | 0.0148 rad | 0.0389 rad | true が良い |

circle 中の XY RMS は、`linear_mode=false` で約 78% 改善した。

### Gimbal 角

| phase | 指標 | linear_mode=true | linear_mode=false | 評価 |
| --- | ---: | ---: | ---: | --- |
| hover | \|phi\| RMS | 0.0360 rad | 0.0234 rad | false が良い |
| circle | \|phi\| RMS | 0.0299 rad | 0.0230 rad | false が良い |
| post-hover | \|phi\| RMS | 0.0340 rad | 0.0269 rad | false が良い |
| circle | 最大 gimbal step | 0.0891 rad | 0.0236 rad | false がかなり滑らか |
| circle | jump > 0.2 rad | 0 回 | 0 回 | 両方問題なし |
| circle | jump > 0.5 rad | 0 回 | 0 回 | 両方問題なし |

`linear_mode=false` のほうが gimbal 角が小さく、特に circle 中の角度変化が滑らかだった。

### Thrust

| phase | 指標 | linear_mode=true | linear_mode=false | 評価 |
| --- | ---: | ---: | ---: | --- |
| hover | total thrust mean | 32.371 N | 32.362 N | 同等 |
| circle | total thrust mean | 32.349 N | 32.344 N | 同等 |
| post-hover | total thrust mean | 32.351 N | 32.349 N | 同等 |
| circle | max single thrust | 9.381 N | 9.200 N | false が少し低い |
| circle | thrust sum square mean | 263.328 | 263.187 | false が少し低い |

推力総量はほぼ同じ。`linear_mode=false` は single rotor の最大推力と二乗和が少し低く、配分としてもやや良い。

### NLOpt

`linear_mode=false` では NLOpt/SLSQP が使われ、今回の軌道では全区間で成功した。

| phase | success rate | fallback/fail | iteration mean | iteration p95 | iteration max |
| --- | ---: | ---: | ---: | ---: | ---: |
| hover | 100% | 0 | 8.26 | 12 | 17 |
| circle | 100% | 0 | 8.34 | 14 | 20 |
| post-hover | 100% | 0 | 8.25 | 13 | 17 |

今回の条件では、`linear_mode=false` で局所解に落ちて破綻するような挙動は bag 上では見えなかった。

## 注意点

- これは 1 回の simulation 結果なので、最終判断には複数回試行したほうがよい。
- `linear_mode=false` の初回 launch で Gazebo が一度 segfault したが、再起動後は正常に takeoff/circle/land まで完走した。
- `linear_mode=true` 側にも `/delta/debug/nlopt_result` は publish されていたが、linear 経路では NLOpt を呼ばないため、この値は評価対象外にした。
- コード上は `linear_mode || first_run_` のとき `linearWrenchAllocation()`、それ以外で `nonlinearWrenchAllocation()` を呼ぶ。
- `linear_mode=false` でも NLOpt が失敗した場合は linear allocation に fallback する。

## 今回の判断

今回の hover/circle/yaw 軌道では、以下の理由で **`linear_mode=false` を推奨**する。

- circle 中の XY tracking が大幅に良い。
- gimbal 角が小さい。
- gimbal の角度変化が滑らか。
- NLOpt の成功率が 100% で fallback が発生していない。
- 推力配分も同等以上。

ただし yaw error は `linear_mode=true` のほうが小さいので、yaw 精度を厳しく見る実験では追加評価が必要。
