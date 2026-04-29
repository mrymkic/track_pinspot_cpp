# track_pinspot_cpp

Azure Kinect 2台を使った同期計測と、`base` / `aux` の両方で body tracking を動かしながら座標補正を検証するための C++ プロジェクトです。

現時点の主方針は、次の構成です。

- `base`: body tracking を実行する主センサ
- `aux`: 有線同期された補助センサ。`enable_aux_body_tracking=true` のとき body tracking も実行
- 最終融合: `base` の `x, y` と `aux` 由来の `z` を組み合わせる
- `aux` の骨格が取れたときは `base_xy + aux_body_z` を優先し、取れないときは `base_xy + aux_depth_z` へフォールバックする

## 現在の到達点

- Azure Kinect 2台の `MASTER / SUBORDINATE` 有線同期は確認済み
- アプリ内でも `phase_delta_us` / `phase_error_us` により同期状態を監視できる
- `enable_aux_body_tracking` により `base` / `aux` の 2台同時 body tracking を有効化
- 最新の確認では、`Base body tracker created ...` / `Aux body tracker created ...` の後も同期ログが継続し、以前の `aux capture stopped updating` は再現していない
- `aux` tracker も新しい capture 世代だけ enqueue するようにし、同一フレームの詰まりを避ける
- `aux` capture stall 時には、aux カメラ再起動と aux tracker 再生成を試みる
- `gpu_cuda + dnn_model_2_0_lite_op11.onnx` が、現時点で最も安定していた構成
- `aux` 側の骨格が取れない場面では、従来どおり depth ベース補正へフォールバックできる
- ただし `aux_translation_mm` と `rotation_matrix` はまだダミー値であり、融合後の空間精度は未検証

## 背景

当初は `base` と `aux` の両方で body tracking を起動し、それぞれの骨格結果を融合する構成を想定していました。

しかし検証の結果、次の問題がありました。

- `aux` 側の `k4abt_tracker_create()` が安定しない
- 2台同期 capture はできても、GPU body tracking の負荷下で `aux` の更新が止まりやすい
- 特に `gpu_directml` や `gpu_cuda + full model` では、`aux capture stopped updating` が出やすかった

そのため一時期は、

- `base`: body tracking
- `aux`: depth-only

という役割分担に切り替えていました。

その後、

- `aux` tracker も `base` と同様に capture generation 単位で進める
- stall 時は `aux` カメラ再起動後に `aux` tracker も作り直す

ように修正し、現在は `enable_aux_body_tracking` を有効にしたとき、

- `aux` 骨格あり: `base_xy + aux_body_z`
- `aux` 骨格なし: `base_xy + aux_depth_z`

の順で使い分ける構成に戻しています。

つまり現状は、「2台同時に body tracking を動かすところまではできているが、融合用の外部パラメータはまだ仮」という段階です。

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

## 2台同時 body tracking の現状

このプロジェクトでは何度か方針を切り替えています。

### 1. 1台版

- `track_test.cpp`
- 単体の Azure Kinect で body tracking を行う版

### 2. 初期の 2台同時 body tracking の試行

- `track_test_2.cpp` をベースに 2 台同時 tracking を試行
- しかし `aux` 側 tracker 作成と継続動作が不安定

### 3. 一時的に `base body tracking + aux depth-only` へ移行

- `base` が人物の関節を検出
- `aux` は body tracking をせず、投影先近傍の depth を取得
- `aux` の depth から `z` 成分だけを補正

この方式なら、2台同時 body tracking より負荷と不安定要因を減らしつつ、奥行き補正の恩恵を受けられます。

### 4. 現在

- `base` と `aux` の tracker は並行稼働できている
- `aux` 骨格が取れたときは `base_xy + aux_body_z` を使う
- `aux` 骨格が取れないときは `base_xy + aux_depth_z` へフォールバックする
- 融合用の `aux_translation_mm` / `rotation_matrix` はまだダミーのため、座標精度の評価はこれから

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
2. `aux` tracker に同じ関節があれば、その 3D 座標を `base` 座標系へ変換する
3. `aux` 骨格が取れたときは `fused = { base.x, base.y, aux_body由来z }` を作る
4. `aux` 骨格が取れないときは、`base` の 3D 座標を `aux` 座標系へ変換する
5. それを `aux` depth image 上へ投影し、投影先近傍の depth 候補を探索して 3D に復元する
6. 予測位置に最も近い `aux` 由来 3D 点を `base` 座標系へ戻し、`fused = { base.x, base.y, aux_depth由来z }` を作る
7. どちらも使えないときは `base_only` として `base` 単独値を使う

ログでは融合元を次のように表現しています。

- `source=base_xy + aux_body_z`
- `source=base_xy + aux_depth_z`
- `source=base_only`

なお、現状は追従対象を耳位置に絞っており、全関節を対等に融合しているわけではありません。

## この方式でできること / できないこと

### できること

- `base` / `aux` の 2台同時 body tracking を並行稼働させること
- `aux` 骨格が取れたときに `aux_body_z` を使うこと
- `aux` 骨格が取れないときも `aux_depth_z` へフォールバックして処理継続すること
- 耳や頭部など、特定点の `z` 補正を試すこと

### できないこと

- 現時点で信頼できる融合座標を得ること
- 2台の骨格を完全に対等な重みで融合すること
- 全関節について校正済みの 2台融合を行うこと
- `base` が見失った関節を `aux` だけで確実に復元すること

つまり現状の `aux` は、「並行して body tracking も動かす補助センサ」ではあるものの、座標融合の精度面はこれから詰める段階です。

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
- `track_config_2_capture_images.json`
  - チェッカーボード撮影向けの Kinect 画像キャプチャ用
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
- `enable_aux_body_tracking = true`
- `aux` 骨格が不安定な場面では depth 補正へフォールバック
- `aux_translation_mm` / `rotation_matrix` は仮値として扱う
- 同期維持のため、`aux` の color は内部的には有効

## Kinect 画像キャプチャ

チェッカーボード画像を取得したいときは、`track_config_2_capture_images.json` を使って `track_test_cpp_2cam` を起動します。

- `enable_body_tracking = false`
- `enable_image_capture = true`
- `capture_output_dir = "calibration_images"`
- `aux_synchronized_images_only = true`

起動後にチェッカーボードを `base` / `aux` の両方に見せ、`c` キーを押すと現在の画像ペアを保存します。

Windows では `aux` が予約名なので、補助カメラ側の保存フォルダ名は `aux_color` です。

- `base`: `calibration_images/<session>/base/0001.bmp`
- `aux`: `calibration_images/<session>/aux_color/0001.jpg`
- 必要なら `capture_save_aux_depth = true` で `aux_depth` も保存可能
- 保存に成功すると、映像ウィンドウ上に `Saved 0001.bmp / 0001.jpg` の通知が数秒表示されます

`<session>` には実行時刻ベースのセッション名が付きます。保存された `base` と `aux_color` のディレクトリは、そのまま `tools/calibrate_checkerboard_extrinsics.py` の入力に使えます。

例:

```bat
build_2cam_x64\track_test_cpp_2cam.exe track_config_2_capture_images.json
```

## 既知の注意点

- `CUDA provider probe failed ... error 1114` が出ても、その後に `Base body tracker created with mode: gpu_cuda` が出る場合は実運用上 tracker 作成に成功している
- `subordinate_delay_off_master_usec = 160` は subordinate の color capture タイミング設定であり、ログ上の `phase_delta_us` と 1:1 に一致する値ではない
- `raw_delta_us` はフレーム周期の整数倍だけずれて見えることがあるので、同期確認では `phase_delta_us` / `phase_error_us` を優先して見る
- 融合用の `aux_translation_mm` / `rotation_matrix` はまだダミーであり、`source=base_xy + aux_body_z` や `source=base_xy + aux_depth_z` が出ていても、その座標精度までは保証していない
- `aux` stream stall 時は再起動と tracker 再生成を試みるが、USB や電源条件が悪いと再起動に失敗する可能性は残る

## 今後の確認事項

- 実人物を入れた状態で `aux_body=FOUND` と `source=base_xy + aux_body_z` が安定して出るか
- `aux_body_z` と `aux_depth_z` のどちらが実運用で安定するか
- `aux_translation_mm` と `rotation_matrix` を実測値に置き換えたときの精度
- `sample_aux_depth_point_for_base_joint()` の探索条件の最適化
- 長時間運転時の `aux` 再起動経路の安定性

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
| 2026-04-29 | `209fb32` | `aux` capture stall 後の再起動で、serial 再解決・カメラ再初期化・aux tracker 再生成まで行うよう修正。 |
| 2026-04-29 | `3dca973` | `aux` tracker も capture generation 単位で進めるよう修正し、`base_xy + aux_body_z` を優先する 2台同時 body tracking を既定構成へ反映。 |
| 2026-04-28 | `1105d31` | `aux_depth=MISSING` 時に、投影失敗・探索窓内の depth 欠損・3D 候補不足・空間誤差超過などの理由をログへ出す診断を追加。 |
| 2026-04-28 | `51bc322` | sync baseline / phase 診断が同じ stale な aux frame を繰り返し使わないようにし、新しい base / aux frame pair のときだけ学習・判定するよう修正。 |
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
