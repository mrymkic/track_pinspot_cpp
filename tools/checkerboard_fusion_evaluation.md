# チェッカーボード回転による座標統合評価

`evaluate_checkerboard_fusion.py` は、既知角で回転させたチェッカーボード画像から `base` / `aux変換後` / `fused` の座標精度を比較するスクリプトです。

評価の考え方は、`評価手法/kinect深度座標精度の評価手法（補足）.pdf` の流れに合わせています。

1. 基準角度のチェッカーボード画像ペアを撮る
2. パン回転で既知角だけ回した画像ペアを撮る
3. 各画像からチェッカーボード姿勢を推定する
4. 理論上の深度変化量と、Kinect画像から得た変化量を比較する

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

- `base`
  - 基準フレームとの差分で見た深度変化誤差
- `aux_transformed`
  - `aux_translation_mm + rotation_matrix * p_aux` で `base` 座標系へ移した後の誤差
- `fused`
  - `base.x, base.y, aux_in_base.z` として統合した座標の誤差
- `fused_minus_base`
  - `base` と比べて `fused` がどれだけ改善したか

`overall` は全 corner の集計、`by_stem` は角度ごとの集計です。
`fused` は `z` に `aux_in_base.z` をそのまま使うため、深度だけを見る指標は `aux_transformed` と同じ値になります。差が出るのは `x, y` を含む 3D 誤差です。

## 前提と注意

- 既定では、パン回転でチェッカーボードの中央列を固定して回す前提で `rotation_axis = board_y` にしています
- 固定しているのが中央行なら `--rotation-axis board_x` を使ってください
- `pivot_coordinate_mm` を省略した場合は、チェッカーボード中心線を固定軸として扱います
- `aux` 側の corner order が 180 度反転している実験では、`--aux-corner-order reverse` が必要になることがあります
- このスクリプトは理論変化量との比較用です。耳追跡ログの runtime 評価は `evaluate_fused_coordinates.py` の方を使ってください
