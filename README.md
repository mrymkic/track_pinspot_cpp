# track_pinspot_cpp

Azure Kinect 2台を使って、耳の 3D 座標を追跡しながら `base` / `aux` の座標を統合するための C++ プロジェクトです。

現在の主実装は [`track_test_2.cpp`](./track_test_2.cpp) です。  
1台版の実験コードは [`track_test.cpp`](./track_test.cpp) に残しています。

## 最新状態

- 2台の Kinect を `MASTER / SUBORDINATE` で有線同期して使用します。
- `base` と `aux` の両方で body tracking を有効化できます。
- 追跡対象は耳です。複数人が写っていても、選択したカメラで最前面にいる 1 人だけを追跡します。
- 最前面の判定対象カメラは `tracked_person_camera` で切り替えできます。
- 座標統合モードは `fusion_mode` で切り替えできます。
- 補助カメラの表示は白黒 depth ではなくフルカラー表示です。
- チェッカーボード撮影から `aux_translation_mm` / `rotation_matrix` を更新するフローがあります。

## 主要な考え方

### カメラの役割

- `base`
  - 基準座標系になるカメラ
- `aux`
  - `base` に対する補助カメラ
  - チェッカーボードで外部パラメータを合わせてから統合に使います

### 複数人がいるときの対象選択

`tracked_person_camera` で、どちらのカメラを基準に「一番前の人」を選ぶかを決めます。

- `base`
  - `base` カメラ内で最も `z` が小さい耳を選びます
  - その後、`aux` 側の耳候補を `base` 座標系へ変換し、最も近い候補を対応付けます
- `aux`
  - `aux` カメラ内で最も `z` が小さい耳を選びます
  - その後、`base` 側の耳候補から最も近い候補を対応付けます

既定値は `base` です。

### 座標統合モード

`fusion_mode` は次の 2 モードです。

- `depth_only`
  - 既定値
  - 基本は `base` の `x, y` を使い、`aux` 由来の `z` を使います
  - `aux` の body tracking が使えればその `z`、使えなければ `aux` depth から復元した `z` にフォールバックします
- `full_3d`
  - `aux` の 3D 耳座標を `base` 座標系へ変換した値をそのまま使います

「深さだけ統合」を既定にしつつ、必要なら「3D を丸ごと統合」に切り替える構成です。

## 主要設定

通常使う設定ファイルは次の 3 つです。

- [`track_config_2.json`](./track_config_2.json)
  - 通常実行用
- [`track_config_2_bt_cuda_lite_dual_eval.json`](./track_config_2_bt_cuda_lite_dual_eval.json)
  - 融合トレース CSV を出したいときの評価用
- [`track_config_2_capture_images.json`](./track_config_2_capture_images.json)
  - チェッカーボード撮影用

特に重要なキー:

- `enable_body_tracking`
  - body tracking の有効化
- `enable_aux_body_tracking`
  - `aux` 側の body tracking 有効化
- `tracked_ear`
  - `left` / `right`
- `tracked_person_camera`
  - `base` または `aux`
  - 同義語として `main` / `master` は `base`、`sub` / `subordinate` は `aux` として扱います
- `fusion_mode`
  - `depth_only` または `full_3d`
- `body_tracking_mode`
  - 現状の推奨は `gpu_cuda`
- `body_tracking_model_path`
  - 現状の推奨は `dnn_model_2_0_lite_op11.onnx`
- `aux_translation_mm`
- `rotation_matrix`
  - チェッカーボード校正で更新する値

## 推奨設定

現時点では次の組み合わせを基準にするのが無難です。

- `body_tracking_mode = "gpu_cuda"`
- `body_tracking_model_path = "dnn_model_2_0_lite_op11.onnx"`
- `enable_aux_body_tracking = true`
- `tracked_person_camera = "base"`
- `fusion_mode = "depth_only"`

## ビルド

Visual Studio 2022 の x64 開発環境でビルドします。

```bat
build_2cam_x64.cmd
```

生成される主な実行ファイル:

- `build_2cam_x64\track_test_cpp_2cam.exe`
- `build_2cam_x64\export_kinect_rig_calibration.exe`

## 通常実行

```bat
build_2cam_x64\track_test_cpp_2cam.exe track_config_2.json
```

ログには、どのカメラを基準に人選択したかや、どの統合ソースを使ったかが出ます。

例:

- `selection_camera=base`
- `source=base_xy + aux_body_z`
- `source=base_xy + aux_depth_z`
- `source=aux_body_in_base_3d`
- `source=base_only`

## チェッカーボードで座標統合を合わせる手順

詳しくは [`tools/checkerboard_calibration.md`](./tools/checkerboard_calibration.md) を参照してください。  
ここでは最短手順だけまとめます。

### 1. 画像を撮影する

```bat
build_2cam_x64\track_test_cpp_2cam.exe track_config_2_capture_images.json
```

- `c` キーで現在の画像ペアを保存します
- `base` 側は `base/`
- 補助カメラ側は予約語回避のため `aux_color/`
- 必要なら `capture_save_aux_depth = true` で `aux_depth/` も保存できます

### 2. rig calibration を書き出す

```bat
build_2cam_x64\export_kinect_rig_calibration.exe --output .\tools\checkerboard_rig_live.json
```

### 3. 校正設定を編集する

[`tools/checkerboard_calibration_config_template.json`](./tools/checkerboard_calibration_config_template.json) の `SESSION_NAME` などを、今回の撮影内容に合わせて更新します。

### 4. 外部パラメータを推定して設定へ反映する

```powershell
python .\tools\calibrate_checkerboard_extrinsics.py --calibration-config .\tools\checkerboard_calibration_config_template.json
```

コーナー順が逆なら次で再実行します。

```powershell
python .\tools\calibrate_checkerboard_extrinsics.py --calibration-config .\tools\checkerboard_calibration_config_template.json --aux-corner-order reverse
```

`config_out` を `track_config_2.json` にしている場合は、関連する `track_config_2*.json` にも同じ rig 値が同期されます。

## 統合結果の評価

### 実行時トレースを出す

```bat
build_2cam_x64\track_test_cpp_2cam.exe track_config_2_bt_cuda_lite_dual_eval.json
```

CSV は既定で `fusion_eval/fusion_trace.csv` に保存されます。

### CSV を評価する

```powershell
python .\tools\evaluate_fused_coordinates.py --trace-csv .\fusion_eval\fusion_trace.csv
```

チェッカーボード回転治具での評価は次を参照してください。

- [`tools/checkerboard_fusion_evaluation.md`](./tools/checkerboard_fusion_evaluation.md)
- [`tools/evaluate_checkerboard_fusion.py`](./tools/evaluate_checkerboard_fusion.py)

## 主要ファイル

- [`track_test_2.cpp`](./track_test_2.cpp)
  - 2台同期、耳追跡、統合処理の主実装
- [`track_test.cpp`](./track_test.cpp)
  - 1台版の実験コード
- [`tools/calibrate_checkerboard_extrinsics.py`](./tools/calibrate_checkerboard_extrinsics.py)
  - チェッカーボードから外部パラメータを推定
- [`tools/export_kinect_rig_calibration.cpp`](./tools/export_kinect_rig_calibration.cpp)
  - Kinect から rig calibration JSON を書き出し

## 補足

- `aux` を人選択基準にした場合でも、`aux` body tracking が使えなければ自動で `base` 基準にフォールバックします。
- `depth_only` は「完全な点群融合」ではなく、「`base` の横位置を保ちつつ `aux` 由来の奥行きを使う」モードです。
- `full_3d` は `aux` の 3D 点を `base` 座標系へ持ってくるモードです。
- 現状の複数人対応は「最前面の 1 人を選ぶ」段階で、全員同時追跡ではありません。
