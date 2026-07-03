# チェッカーボード回転による座標統合評価

`evaluate_checkerboard_fusion.py` は、既知角で回転させたチェッカーボード画像から `base` / `aux変換後` / `fused` の座標精度を比較するスクリプトです。

評価の考え方は、`評価手法/kinect深度座標精度の評価手法（補足）.pdf` の流れに合わせています。

1. 基準角度のチェッカーボード画像ペアを撮る
2. パン回転で既知角だけ回した画像ペアを撮る
3. 各画像からチェッカーボード姿勢を推定する
4. 理論上の深度変化量と、Kinect画像から得た変化量を比較する
5. あわせて、全フレームの既知角から剛体フィット残差と回転角誤差も評価する

## 入力

- `base` 側チェッカーボード画像ディレクトリ
- `aux` 側チェッカーボード画像ディレクトリ
- `checkerboard_rig_live.json` のような rig calibration JSON
- `track_config_2.json` のような `aux_translation_mm` / `rotation_matrix` を持つ JSON
- 各画像 stem と既知角を対応付けた CSV

角度 CSV の例:

```csv
stem,angle_deg
0001,0
0002,5
0003,10
0004,15
```

`stem` は `base/0001.bmp` と `aux_color/0001.jpg` の `0001` に相当します。

## 実行例

テンプレート JSON を使う場合:

```powershell
python .\tools\evaluate_checkerboard_fusion.py --evaluation-config .\tools\checkerboard_fusion_eval_config_template.json
```

CLI 引数だけで実行する場合:

```powershell
python .\tools\evaluate_checkerboard_fusion.py `
  --base-dir .\calibration_images\SESSION_NAME\base `
  --aux-dir .\calibration_images\SESSION_NAME\aux_color `
  --rig-calibration .\tools\checkerboard_rig_live.json `
  --transform-config .\track_config_2.json `
  --angles-csv .\calibration_images\SESSION_NAME\angles.csv `
  --board-cols 9 `
  --board-rows 6 `
  --square-size-mm 32 `
  --aux-corner-order reverse `
  --reference-stem 0001 `
  --rotation-axis board_y `
  --output-json .\calibration_images\SESSION_NAME\checkerboard_fusion_eval.json
```

## 主な出力

- `base_anchored`
  - 従来方式です。`reference_stem` の `base` 姿勢から理論点群を作るため、`base` が暗黙に基準になります
- `rigid_fit`
  - 全フレームの既知角を使って 1 つの剛体姿勢をフィットした残差です
  - 主に `rigid_fit.fused_minus_base.absolute_3d_rmse_mm_delta` と `absolute_z_rmse_mm_delta` を見ます
- `angle_only`
  - `rigid_fit` で得た姿勢から各フレームの回転角を再推定し、既知角との差を見ます
  - 主に `angle_only.fused_minus_base.angle_rmse_deg_delta` を見ます

トップレベルの `overall` / `by_stem` は互換性のため従来の `base_anchored` 集計を残しています。
`fused` は `z` に `aux_in_base.z` をそのまま使うため、legacy の深度だけを見る指標は `aux_transformed` と同じ値になります。差が出るのは `x, y` を含む 3D 誤差か、回転角の整合性です。

## 前提と注意

- 既定では、パン回転でチェッカーボードの中央列を固定して回す前提で `rotation_axis = board_y` にしています
- 固定しているのが中央行なら `--rotation-axis board_x` を使ってください
- `pivot_coordinate_mm` を省略した場合は、チェッカーボード中心線を固定軸として扱います
- `aux` 側の corner order が 180 度反転している実験では、`--aux-corner-order reverse` が必要になることがあります
- `angle_only` は絶対角の定数オフセットと回転方向の符号が不定なので、各ラベルごとに `reference_stem` と向きを既知角へ合わせてから誤差を出しています
- このスクリプトはチェッカーボード治具での相対評価用です。耳追跡ログの runtime 評価は `evaluate_fused_coordinates.py` の方を使ってください
