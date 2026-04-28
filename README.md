# track_pinspot_cpp

Azure Kinect 2台を使った同期計測と、`base` 側 body tracking と `aux` 側 depth を組み合わせた座標補正を検証するための C++ プロジェクトです。

現時点の主方針は、2台同時 body tracking ではなく、次の構成です。

- `base`: body tracking を実行する主センサ
- `aux`: 有線同期された補助センサ。body tracking は行わず depth のみを利用
- 最終融合: `base` の `x, y` と `aux` 由来の `z` を組み合わせる

## 現在の到達点

- Azure Kinect 2台の `MASTER / SUBORDINATE` 有線同期は確認済み
- アプリ内でも `phase_delta_us` / `phase_error_us` により同期状態を監視できる
- `base body tracking + aux depth-only` 構成が現実的な方式として成立
- `gpu_cuda + dnn_model_2_0_lite_op11.onnx` が、現時点で最も安定していた構成
- `enable_aux_body_tracking` を使うと、2台同時 body tracking を診断モードとして再度有効化できる

## 背景

当初は `base` と `aux` の両方で body tracking を起動し、それぞれの骨格結果を融合する構成を想定していました。

しかし検証の結果、次の問題がありました。

- `aux` 側の `k4abt_tracker_create()` が安定しない
- 2台同期 capture はできても、GPU body tracking の負荷下で `aux` の更新が止まりやすい
- 特に `gpu_directml` や `gpu_cuda + full model` では、`aux capture stopped updating` が出やすかった

そのため現在は、2台同時 body tracking ではなく、

- `base`: body tracking
- `aux`: depth-only

という役割分担に切り替えています。

## 同期について

### ハードウェア同期

2台の Azure Kinect は `MASTER / SUBORDINATE` の有線同期で動作させます。

- `base`: `MASTER`
- `aux`: `SUBORDINATE`
- `subordinate_delay_off_master_usec = 160`

`k4arecorder` でも subordinate を先に起動し、その後 master を起動することで録画開始できることを確認しています。

### アプリ内の同期判定

アプリ内では depth timestamp から次の値を計算しています。

- `raw_delta_us`
  - `aux_depth_ts_us - base_depth_ts_us`
- `phase_delta_us`
  - `raw_delta_us` を1フレーム周期で折りたたんだ位相差
- `baseline_phase_delta_us`
  - 初期フレームから学習した基準位相差
- `phase_error_us`
  - 現在の位相差が基準からどれだけずれているか

重要なのは `raw_delta_us` そのものではなく、`phase_error_us` が小さい範囲で安定していることです。

例えば `raw_delta_us` が

- `11933`
- `45267`
- `-21400`

と見えても、30fps の周期 `33333us` で折りたたむと、どれもほぼ同じ位相として扱えます。

## なぜ2台同時 body tracking を採用していないか

このプロジェクトでは何度か方針を切り替えています。

### 1. 1台版

- `track_test.cpp`
- 単体の Azure Kinect で body tracking を行う版

### 2. 2台同時 body tracking の試行

- `track_test_2.cpp` をベースに 2 台同時 tracking を試行
- しかし `aux` 側 tracker 作成と継続動作が不安定

### 3. `base body tracking + aux depth-only` へ移行

- `base` が人物の関節を検出
- `aux` は body tracking をせず、投影先近傍の depth を取得
- `aux` の depth から `z` 成分だけを補正

この方式なら、2台同時 body tracking より負荷と不安定要因を減らしつつ、奥行き補正の恩恵を受けられます。

## body tracking モードの切り分け結果

検証時の傾向は次の通りです。

- `capture-only`: 安定
- `body_tracking_mode=cpu`: 安定
- `gpu_directml`: 不安定
- `gpu_cuda + full model`: 不安定
- `gpu_cuda + lite model`: 安定化傾向が強い

また、DXGI adapter の確認により `gpu_device_id=0` は `NVIDIA GeForce RTX 3060 Laptop GPU` を指していることを確認しています。

そのため、現在の推奨は次です。

- `body_tracking_mode = gpu_cuda`
- `gpu_device_id = 0`
- `body_tracking_model_path = dnn_model_2_0_lite_op11.onnx`

## 現在の座標融合方針

現在の融合は、2台の骨格を平均する方式ではありません。

処理の流れは次の通りです。

1. `base` で対象関節の 3D 座標を取得する
2. その 3D 座標を `aux` 座標系へ変換する
3. `aux` depth image 上へ投影する
4. 投影先近傍の depth 候補を探索し、3D に復元する
5. 予測位置に最も近い `aux` 由来 3D 点を採用する
6. その点を `base` 座標系へ戻す
7. 最終的に `fused = { base.x, base.y, aux由来z }` を作る

ログではこの融合元を `source=base_xy + aux_depth_z` と表現しています。

## この方式でできること / できないこと

### できること

- 耳や頭部など、特定点の奥行き補正
- `base` 単独より安定した `z` 推定
- `base` を主とした軽量な2台融合

### できないこと

- `aux` 単独で関節を再認識すること
- 2台の骨格を完全に対等に融合すること
- `base` が見失った関節を `aux` だけで復元すること

つまり、`aux` は独立した認識器ではなく、`base` の補助 depth センサとして使っています。

## 主要ファイル

- `track_test.cpp`
  - 1台版の body tracking 実験
- `track_test_2.cpp`
  - 2台同期・融合実験の主実装
- `track_config.json`
  - 1台版の設定
- `track_config_2.json`
  - 現行の推奨設定
- `track_config_2_bt_cpu.json`
  - CPU body tracking 診断用
- `track_config_2_bt_cuda.json`
  - CUDA + full model 診断用
- `track_config_2_bt_cuda_lite.json`
  - CUDA + lite model 診断用
- `track_config_2_bt_cuda_dual.json`
  - CUDA + full model で `base + aux` 同時 body tracking を試す診断用
- `track_config_2_bt_cuda_lite_dual.json`
  - CUDA + lite model で `base + aux` 同時 body tracking を試す診断用
- `track_config_2_capture_only.json`
  - body tracking 無効の capture-only 切り分け用
- `measure_vram_bodytracking.ps1`
  - `nvidia-smi` ベースの VRAM サンプリング補助スクリプト
- `CMakeLists.txt`
  - ビルド設定と runtime DLL staging

## ビルド

Visual Studio 2022 の x64 開発環境でビルドします。

例:

```bat
cmd.exe /c ""C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64 && "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B .\build_2cam_x64 -G Ninja && "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build .\build_2cam_x64 --target track_test_cpp_2cam"
```

## 現在の推奨実行条件

- body tracking: `gpu_cuda`
- model: `dnn_model_2_0_lite_op11.onnx`
- `base`: `MASTER`
- `aux`: `SUBORDINATE`
- `aux` は depth-only 用途
- ただし同期維持のため、`aux` の color は内部的には有効

## 既知の注意点

- `CUDA provider probe failed ... error 1114` が出ても、その後に `Base body tracker created with mode: gpu_cuda` が出る場合は実運用上 tracker 作成に成功している
- `subordinate_delay_off_master_usec = 160` は subordinate の color capture タイミング設定であり、ログ上の `phase_delta_us` と 1:1 に一致する値ではない
- 安定動作していても、最終的には `aux_depth=FOUND` が安定して出るかどうかを別途確認する必要がある

## 今後の確認事項

- 実人物を入れた状態で `aux_depth=FOUND` が安定するか
- `source=base_xy + aux_depth_z` による補正が安定して働くか
- `aux_translation_mm` と `rotation_matrix` の精度
- `sample_aux_depth_point_for_base_joint()` の探索条件の最適化

## VRAM 計測

body tracking 時の GPU メモリ使用量を追うために、次の補助ファイルを追加しています。

- `measure_vram_bodytracking.ps1`
  - `nvidia-smi` を一定間隔でサンプリングし、CSV に保存する
- `track_config_2_bt_cuda.json`
  - `base` のみ、`gpu_cuda + full model`
- `track_config_2_bt_cuda_lite.json`
  - `base` のみ、`gpu_cuda + lite model`
- `track_config_2_bt_cuda_dual.json`
  - `base + aux` の2台同時 body tracking 診断用、`gpu_cuda + full model`
- `track_config_2_bt_cuda_lite_dual.json`
  - `base + aux` の2台同時 body tracking 診断用、`gpu_cuda + lite model`

### 計測例

1台 tracker の既定計測:

```bat
powershell -ExecutionPolicy Bypass -File .\measure_vram_bodytracking.ps1
```

2台同時 tracker の lite model 計測:

```bat
powershell -ExecutionPolicy Bypass -File .\measure_vram_bodytracking.ps1 -ConfigPath ..\track_config_2_bt_cuda_lite_dual.json -OutputCsv .\build_2cam_x64\vram_samples_dual_lite.csv
```

2台同時 tracker の full model 計測:

```bat
powershell -ExecutionPolicy Bypass -File .\measure_vram_bodytracking.ps1 -ConfigPath ..\track_config_2_bt_cuda_dual.json -OutputCsv .\build_2cam_x64\vram_samples_dual_full.csv
```

### 見るべき列

- `total_memory_used_mib`
- `delta_from_baseline_mib`
- `gpu_util_percent`
- `memory_util_percent`
- `target_pid`
- `target_process_memory_mib`

この PC は WDDM 環境のため、`nvidia-smi` のプロセス別 VRAM が `N/A` や空欄になることがあります。その場合は、まず `delta_from_baseline_mib` を主指標として比較します。

## 変更履歴

重要な変更は、コミット単位で次の表に残します。

| Date | Commit | Summary |
| --- | --- | --- |
| 2026-04-28 | 51bc322 | sync baseline / phase 診断が同じ stale な ux frame を繰り返し使わないようにし、新しい ase / ux frame pair のときだけ学習・判定するよう修正。 |
| 2026-04-28 | `89fbe9e` | `track_test_2.cpp` に capture pump の generation・最終 depth timestamp・timeout/failure 回数の診断ログを追加し、stream stall の切り分けをしやすくした。 |
| 2026-04-28 | `561a88f` | `measure_vram_bodytracking.ps1` の `$PID` 予約変数衝突を解消し、WDDM 環境でもサンプリングが途中で落ちないようにした。 |
| 2026-04-28 | `79e4df3` | `measure_vram_bodytracking.ps1` が WDDM 環境の `[N/A]` を安全に扱えるよう修正し、VRAM CSV 取得を継続できるようにした。 |
| 2026-04-28 | `44a4b86` | `enable_aux_body_tracking` を追加し、`gpu_cuda` で2台同時 body tracking を直接測るための dual 診断 config を追加。 |
| 2026-04-28 | `bfca329` | body tracking 実行時の PID ログと `measure_vram_bodytracking.ps1` を追加し、VRAM サンプリングを自動化。 |
| 2026-04-28 | `71f35ed` | README と `.gitignore` を整理し、GitHub 上で読みやすい構成に修正。 |
| 2026-04-28 | `f98dd1f` | 2カメラ同期・body tracking 切り分け内容を含む初回スナップショットを登録。 |

今後も、GitHub に push する大きめの変更はこの表に追記していく想定です。

## Git 管理メモ

このリポジトリでは、ビルド成果物や検証ログを Git に含めないようにしています。

主に ignore しているもの:

- `.vs/`
- `build*/`
- `CMakeFiles/`
- `*.obj`, `*.pdb`, `*.exe`, `*.dll`
- `*.mkv`
- runtime log

GitHub 上では、この README を起点に `track_test_2.cpp`、`CMakeLists.txt`、`track_config_2*.json` を追うと、現在の構成を把握しやすいです。
