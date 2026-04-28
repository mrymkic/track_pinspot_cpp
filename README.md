# track_pinspot_cpp

Azure Kinect と PTU を使った追従型ピンスポット制御プロジェクトです。  
現在は **Azure Kinect 2台の有線同期** と **`base` 側 body tracking + `aux` 側 depth-only 補正** を主軸に整理しています。

## プロジェクト概要

- `track_test.cpp`
  1台構成の既存版です。body tracking の基準実装として使います。
- `track_test_2.cpp`
  2台構成の現行実装です。`base` のみ body tracking、`aux` は depth-only で使います。
- `track_config_2.json`
  現在の推奨設定です。`gpu_cuda + dnn_model_2_0_lite_op11.onnx` を使います。
- `track_config_2_bt_cpu.json`
  CPU body tracking で切り分けるための設定です。
- `track_config_2_bt_cuda.json`
  CUDA + フル model の切り分け設定です。
- `track_config_2_bt_cuda_lite.json`
  CUDA + lite model の切り分け設定です。
- `track_config_2_capture_only.json`
  body tracking を切って 2台同期 capture のみ確認する設定です。

## 現在の到達点

- 2台の Azure Kinect を `MASTER / SUBORDINATE` で有線同期するところまでは確認済みです。
- `k4arecorder` でも subordinate を先に起動し、master を後起動すると同期録画できることを確認しています。
- アプリ内でも `phase_delta_us` / `phase_error_us` を使った同期診断を入れており、安定ログでは位相差が小さく保たれています。
- 現時点で最も安定している条件は、`gpu_cuda + dnn_model_2_0_lite_op11.onnx` です。
- 2台同時 body tracking は安定しなかったため、現在は `base body tracking + aux depth-only` 構成に落ち着いています。

## ここまでの主な変更経緯

### 1. 1台版から2台版へ拡張

元々は `track_test.cpp` の 1台 body tracking 実装を基に、2台の Azure Kinect を同時に使う構成へ拡張しました。

### 2. 2台同時 body tracking を試行

当初は `base` と `aux` の両方で body tracking を行い、両者の骨格結果を融合する方針でした。  
しかし `track_test_2.cpp` では `aux` 側の `k4abt_tracker_create()` が安定せず、この方針は採用しないことにしました。

### 3. 方針転換: `base` のみ body tracking、`aux` は depth-only

その後、`base` だけで人物追跡を行い、`aux` は body tracking をせず depth だけ使う構成へ切り替えました。  
目的は、`base` が推定した 3D 関節位置に対して、`aux` 側 depth から奥行きを補正することです。

### 4. 2台同期の確認と同期診断の整備

アプリ実行時に `aux` が timeout することがあったため、最初はハードウェア同期失敗を疑いました。  
しかし `k4arecorder` による確認で有線同期自体は成立していることが分かりました。

そのうえで、アプリ内には次の同期診断を追加しました。

- `raw_delta_us`
  `aux_depth_ts_us - base_depth_ts_us`
- `phase_delta_us`
  1フレーム周期で折りたたんだ位相差
- `baseline_phase_delta_us`
  初期フレームから学習した基準位相差
- `phase_error_us`
  現在の位相差が基準からどれだけ外れているか

この診断により、「同期そのものは取れているが、ストリーム継続性が崩れることがある」状態を切り分けられるようになりました。

### 5. capture 取得方式の改善

body tracking や描画の負荷で sensor 側の capture が詰まらないよう、`LatestCapturePump` を導入しました。

- `aux` は専用スレッドで常時 `get_capture()`
- `base` も専用スレッドで常時 `get_capture()`
- main ループでは最新 capture を参照
- body tracker には「新しい base frame だけ enqueue」する方式

これにより、2台の sensor I/O を main ループから分離しています。

### 6. GPU モードの切り分け

body tracking 条件ごとに安定性を比較した結果は次の通りです。

- `capture-only`: 安定
- `body_tracking_mode=cpu`: 安定
- `gpu_directml`: 不安定
- `gpu_cuda + フル model`: 不安定
- `gpu_cuda + lite model`: 安定化

現在の推奨条件は、`gpu_cuda + dnn_model_2_0_lite_op11.onnx` です。

### 7. CUDA 実行環境の整備

`gpu_cuda` を使うため、CMake 側で以下の DLL を build 出力へコピーするようにしています。

- `onnxruntime_providers_cuda.dll`
- `cudart64_110.dll`
- `cublas64_11.dll`
- `cublasLt64_11.dll`
- `cudnn64_8.dll`
- `cudnn_ops_infer64_8.dll`
- `cudnn_cnn_infer64_8.dll`
- `cufft64_10.dll`
- `nvrtc64_112_0.dll`
- `nvrtc-builtins64_114.dll`

また、起動時に CUDA 依存 DLL の存在を診断するログも追加しています。

## 現在の座標融合方針

現在は 2台の骨格結果を平均するのではなく、次の方針で統合します。

1. `base` で対象関節を body tracking する
2. `base` の 3D 点を `aux` 座標系へ変換する
3. その 3D 点を `aux` depth image 上へ投影する
4. 投影先近傍の depth を探索し、候補を 3D に復元する
5. 予測点に最も近い `aux` 側 3D 点を採用する
6. それを `base` 座標系へ戻す
7. 最終的に `fused = { base.x, base.y, aux由来z }` とする

ログ上ではこの融合元を `source=base_xy + aux_depth_z` と表示します。

要するに、`base` が横方向・上下方向を担い、`aux` が奥行き方向を補正する構成です。

## 現在の推奨実行条件

- body tracking: `gpu_cuda`
- GPU: `gpu_device_id = 0`
- model: `dnn_model_2_0_lite_op11.onnx`
- `base`: `MASTER`
- `aux`: `SUBORDINATE`
- `aux` は depth-only で利用
- ただし有線同期維持のため `aux` の color は内部的には有効

推奨 config は `track_config_2.json` です。

## 既知の注意点

- `CUDA provider probe failed ... error 1114` が出ても、その直後に `Base body tracker created with mode: gpu_cuda` が出る場合は、実際には CUDA 経路で tracker 作成に成功していることがあります。
- `subordinate_delay_off_master_usec = 160` は subordinate の color capture タイミング設定であり、ログの `phase_delta_us` と 1:1 で一致する値ではありません。
- `phase_delta_us` は「位相差」、`phase_error_us` は「その位相差が基準からどれだけずれたか」を表しています。

## まだ未完了の項目

同期維持はかなり安定しましたが、最終確認はまだ残っています。

- 実人物を入れた状態で `aux_depth=FOUND` が安定して出るか
- `source=base_xy + aux_depth_z` で本当に奥行き補正が効いているか
- `aux_translation_mm` / `rotation_matrix` が十分正しいか
- `sample_aux_depth_point_for_base_joint()` の探索条件が最適か

## ビルドメモ

- x64 でビルドする前提です。
- Visual Studio 2022 の x64 開発環境を使う必要があります。
- CMake は Azure Kinect SDK / Body Tracking SDK / ONNXRuntime / CUDA Toolkit を自動検出するように調整しています。

例:

```bat
cmd.exe /c ""C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64 && "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B .\build_2cam_x64 -G Ninja && "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build .\build_2cam_x64 --target track_test_cpp_2cam"
```

## Git 管理を始めるときのメモ

このディレクトリを Git 管理する場合は、少なくとも以下は ignore 推奨です。

- `.vs/`
- `build*/`
- `*.pdb`
- 実行ログや一時生成物

README は「なぜ現在の構成に落ち着いたのか」を追えるように書いてあります。  
最初のコミットでは、まずこの README と現行の `track_test_2.cpp` / `CMakeLists.txt` / `track_config_2*.json` を一緒に残すと履歴が追いやすくなります。
