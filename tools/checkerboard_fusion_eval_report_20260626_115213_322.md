# チェッカーボード座標統合評価レポート

## 概要

- 校正セッション: `20260626_113738_305`
- 評価セッション: `20260626_115213_322`
- 評価コマンド:

```powershell
python .\tools\evaluate_checkerboard_fusion.py --evaluation-config .\tools\checkerboard_fusion_eval_config_template.json
```

- 評価前提: チェッカーボードは中央列を軸に回転したものとして `rotation_axis = board_y` を使用
- 基準フレーム: `0001`
- 角度条件: `0, 5, 10, 15, 20 deg`

## 座標統合に使った校正結果

- 使用した設定: `track_config_2.json`
  - `aux_translation_mm = [856.65724, 14.481862, 382.153078]`
  - `rotation_matrix = [[-0.020777, 0.087828, -0.995919], [-0.127358, 0.987786, 0.089767], [0.991639, 0.128704, -0.009338]]`
- 元の校正結果: `calibration_images/20260626_113738_305/checkerboard_result.json`
  - `pairs_used = 6`
  - `pairs_rejected_outliers = []`
  - `translation_std_mm = [1.368756, 0.314331, 0.452119]`

## 全体結果

| 指標 | Base | Aux transformed | Fused |
| --- | ---: | ---: | ---: |
| Z変化 MAE [mm] | 16.054 | 19.407 | 19.407 |
| Z変化 RMSE [mm] | 22.636 | 28.551 | 28.551 |
| 絶対Z RMSE [mm] | 22.636 | 28.632 | 28.632 |
| 絶対3D RMSE [mm] | 25.170 | 64.243 | 30.674 |

`fused_minus_base`:

- `z_change_mae_mm_delta = +3.353`
- `z_change_rmse_mm_delta = +5.914`
- `absolute_z_rmse_mm_delta = +5.996`
- `absolute_3d_rmse_mm_delta = +5.505`

評価の見方:

- `delta` が負なら `fused` が `base` より良いことを意味します。
- 今回はすべて正なので、この評価セットでは `fused` は `base` を上回りませんでした。
- `fused` の Z 系指標が `aux_transformed` と同じなのは、統合後の Z に `aux_transformed.z` をそのまま使っているためです。
- 一方で 3D RMSE は `x, y` に `base` を使うので、`aux_transformed` よりは大きく改善しています。

## 角度ごとの結果

| Stem | 角度 [deg] | Base Z RMSE [mm] | Fused Z RMSE [mm] | 所見 |
| --- | ---: | ---: | ---: | --- |
| `0001` | 0 | 0.000 | 0.000 | 基準フレーム |
| `0002` | 5 | 9.453 | 9.291 | `fused` がわずかに改善 |
| `0003` | 10 | 18.546 | 18.258 | `fused` がわずかに改善 |
| `0004` | 15 | 27.860 | 48.562 | `aux` 側が大きく崩れて悪化 |
| `0005` | 20 | 36.777 | 36.024 | `fused` がわずかに改善 |

今回の全体悪化は `0004`（15度）の影響が支配的です。このフレームだけ `aux_transformed` と `fused` の誤差が急増しており、全体の RMSE を押し上げています。

## 結論

- `20260626_113738_305` で求めた統合変換を、別撮りの回転評価セッション `20260626_115213_322` に適用して評価できました。
- `0-20 deg` の今回データでは、深度変化の精度は `fused` が `base` を上回りませんでした。
- 次に確認したい点は次の 3 つです。
  - `aux_color/0004.jpg` のチェッカーボード検出品質
  - 実際の回転軸が `board_y` 仮定と一致しているか
  - 角度点数を増やした再撮影でも `15 deg` 付近で同じ落ち込みが再現するか
