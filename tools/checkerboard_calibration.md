# チェッカーボードによるキャリブレーション手順

`calibrate_checkerboard_extrinsics.py` は、`track_test_2.cpp` が期待している次の変換を推定します。

```text
p_base_depth = aux_translation_mm + rotation_matrix * p_aux_depth
```

つまり、このスクリプトは `aux depth -> base depth` の変換を求め、最終的に次の設定値として使える形で出力します。

- `aux_translation_mm`
- `rotation_matrix`

## 必要な入力

1. `base` カメラ側のチェッカーボード画像ディレクトリ
2. `aux` カメラ側のチェッカーボード画像ディレクトリ
3. `checkerboard_rig_template.json` を元に作成した rig calibration JSON
4. 必要に応じて `checkerboard_calibration_config_template.json` を元に作成した calibration config JSON

画像ペアはファイル名の `stem` で対応付けます。

```text
base/0001.png <-> aux/0001.png
base/0002.png <-> aux/0002.png
```

ここで `stem` は拡張子を除いたファイル名です。

```text
base/0001.png -> stem = 0001
aux/0001.jpg  -> stem = 0001
```

そのため、`0001.png` と `0001.jpg` はペアになりますが、`base_0001.png` と `aux_0001.png` はファイル名そのものが一致しないためペアになりません。

rig calibration JSON には次の情報が必要です。

- 各デバイスの color camera matrix
- 各デバイスの color distortion coefficients
- `color_to_depth_*` または `depth_to_color_*` のどちらか

このスクリプトは次の前提で動作します。

- チェッカーボード検出は color 画像上で行う
- body tracking / fusion で使う座標系は各デバイスの depth 座標系である

そのため、まず color カメラ座標系でチェッカーボード姿勢を推定し、その結果を depth 座標系に変換してから、最終的な `aux -> base` 変換を求めます。

## 依存関係

次の Python パッケージが必要です。

- `numpy`
- `opencv-python`

例:

```bat
python -m pip install numpy opencv-python
```

## Kinect画像の撮影方法

チェッカーボード画像を新しく撮影したい場合は、`track_test_2.cpp` の画像キャプチャ機能を使えます。

1. `track_config_2_capture_images.json` を指定して `track_test_cpp_2cam` を起動します
2. チェッカーボードを `base` / `aux` の両方に見えるように配置します
3. `c` キーを押すと、その時点の画像ペアが保存されます
4. 保存に成功すると、映像ウィンドウ上に保存したファイル名の通知が数秒表示されます

保存先は `capture_output_dir` で指定したディレクトリ配下に、実行ごとのサブディレクトリとして作成されます。

```text
calibration_images/
  20260429_153012_123/
    base/
      0001.bmp
      0002.bmp
    aux_color/
      0001.jpg
      0002.jpg
```

Windows では `aux` が予約名なので、補助カメラ側の保存フォルダ名は `aux_color` にしています。

この `base` と `aux_color` のディレクトリを、そのまま `calibrate_checkerboard_extrinsics.py` の `--base-dir` / `--aux-dir` に渡せます。

`track_config_2_capture_images.json` では、次の設定をあらかじめ有効にしています。

- `enable_body_tracking = false`
- `enable_image_capture = true`
- `capture_output_dir = "calibration_images"`
- `aux_synchronized_images_only = true`

`aux_synchronized_images_only = true` にしているのは、補助カメラ側でも color/depth が揃った capture を取りやすくし、チェッカーボード撮影を安定させるためです。

## 実行例

### コマンドライン引数だけで実行する場合

```bat
python .\tools\calibrate_checkerboard_extrinsics.py ^
  --base-dir .\calibration_images\SESSION_NAME\base ^
  --aux-dir .\calibration_images\SESSION_NAME\aux_color ^
  --rig-calibration .\tools\checkerboard_rig_template.json ^
  --board-cols 9 ^
  --board-rows 6 ^
  --square-size-mm 25 ^
  --output-json .\calibration_images\SESSION_NAME\checkerboard_result.json ^
  --config-in .\track_config_2.json ^
  --config-out .\track_config_2_checkerboard.json
```

### calibration config JSON を使う場合

`checkerboard_calibration_config_template.json` を編集してから、次のように実行します。

```bat
python .\tools\calibrate_checkerboard_extrinsics.py ^
  --calibration-config .\tools\checkerboard_calibration_config_template.json
```

この config には次の内容をまとめて書けます。

- 画像ディレクトリのパス
- rig calibration JSON のパス
- チェッカーボードの `board_cols`, `board_rows`, `square_size_mm`
- 任意のレポート出力先
- 任意の入出力 tracking config パス

CLI 引数と config JSON の両方に同じ項目がある場合は、CLI 引数の値が優先されます。

## 出力

スクリプトは次の情報を表示します。

- 推定した `aux_translation_mm`
- 推定した `rotation_matrix`
- 平均再投影誤差
- 各画像ペアごとの平行移動 / 回転の診断情報

`--config-in` と `--config-out` を指定した場合は、通常の実行で使えるように `aux_translation_mm` と `rotation_matrix` を反映した tracking config JSON もあわせて出力します。

## 実運用上の注意

- 1枚だけで済ませず、距離や角度を変えた複数のチェッカーボード姿勢で撮影してください
- どちらの画像でもチェッカーボード全体が見えるようにしてください
- 再投影誤差が明らかに悪いフレームは除外してください
- このリポジトリでは `aux_translation_mm` / `rotation_matrix` をまだ調整中の値として扱っているため、config 反映後も実際の body-tracking ログで妥当性を確認してください
