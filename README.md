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
- チェッカーボード撮影から `aux_translation_mm` / `rotation_matrix` を推定し、関連する `track_config_2*.json` へ同期できる
- `calibrate_checkerboard_extrinsics.py` では、180 度反転した別解や誤検出に備えて外れ値ペアを自動除外できる
- 既知角で回したチェッカーボード画像と `evaluate_checkerboard_fusion.py` により、`base` / `aux_transformed` / `fused` の深さ方向変化量を治具ベースで比較できる
- 直近の `fusion_trace.csv` では、`base_vs_aux_body` 平均約 56 mm、`base_vs_aux_depth` 平均約 52 mm まで一致度が改善している
- ただし `reference.csv` は未用意で、実人物の真値に対して `base` より `fused` がどれだけ良いかの絶対証明はまだしていない

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

つまり現状は、「2台同時に body tracking を動かし、チェッカーボードで更新した外部パラメータで統合まで回せているが、真値に対する絶対精度評価はまだ途中」という段階です。

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
- 融合用の `aux_translation_mm` / `rotation_matrix` はチェッカーボード推定値へ更新済みで、runtime CSV と既知角チェッカーボード回転の両方で評価を進められる

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
  - 現行の基準設定。チェッカーボード外部パラメータの反映先
- `track_config_2_bt_cuda_lite_dual_eval.json`
  - CUDA + lite model の 2台同時 body tracking に加えて、融合座標の CSV トレース保存を有効にした評価用
- `track_config_2_capture_images.json`
  - チェッカーボード撮影向けの Kinect 画像キャプチャ用
- `archived_unused_configs_and_scripts/`
  - 現在の運用では使っていない診断用 config と補助スクリプトの退避先

現在 root に残している `track_config_2*.json` は、現行フローで使う最小集合です。`eval` は CSV 保存、`capture_images` は画像保存、`track_config_2.json` は外部パラメータの基準、という役割に絞っています。rig 設定そのものは共通である前提なので、`calibrate_checkerboard_extrinsics.py` で `track_config_2.json` を更新すると、同じディレクトリに残してある関連 `track_config_2*.json` にも `aux_translation_mm` / `rotation_matrix` / Kinect serial / subordinate delay を同期します。過去の診断用 config は `archived_unused_configs_and_scripts/` へ退避しています。
- `tools/calibrate_checkerboard_extrinsics.py`
  - チェッカーボード画像ペアから `aux depth -> base depth` の外部パラメータを推定するスクリプト
- `tools/checkerboard_calibration.md`
  - チェッカーボード撮影から config 反映までの詳細手順
- `tools/evaluate_checkerboard_fusion.py`
  - 既知角で回したチェッカーボード画像ペアから、`base` / `aux_transformed` / `fused` の深さ方向変化誤差を比較するスクリプト
- `tools/checkerboard_fusion_evaluation.md`
  - `evaluate_checkerboard_fusion.py` の入力CSV形式と実行手順
- `tools/evaluate_fused_coordinates.py`
  - 融合座標トレース CSV から、内部整合性と既知座標に対する `base` / `fused` 誤差を評価するスクリプト
- `tools/export_kinect_rig_calibration.cpp`
  - 接続中の Kinect 実機から rig calibration JSON を書き出す補助ツール
- `CMakeLists.txt`
  - ビルド設定と runtime DLL staging

`track_test_cpp_2cam` をビルドすると、上記の現行 `track_config_2*.json` だけが実行ファイルの隣へコピーされます。退避済みの診断用 config は build 出力へはコピーしません。

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
- `aux_translation_mm` / `rotation_matrix` は最新のチェッカーボード推定値を使い、更新後は評価スクリプトで再確認する
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

撮影後は次の順でチェッカーボード外部パラメータを反映します。

1. 接続中の 2 台から rig calibration JSON を書き出します。
2. `tools/checkerboard_calibration_config_template.json` の `SESSION_NAME` を今回の撮影セッション名へ置き換えます。
3. `calibrate_checkerboard_extrinsics.py` を実行して `track_config_2.json` を更新します。

PowerShell 例:

```powershell
build_2cam_x64\export_kinect_rig_calibration.exe --output .\tools\checkerboard_rig_live.json
python .\tools\calibrate_checkerboard_extrinsics.py --calibration-config .\tools\checkerboard_calibration_config_template.json --aux-corner-order reverse
```

`checkerboard_rig_template.json` は見本なので、そのままでは使えません。`color_camera_matrix` が 0 のままだとスクリプトは停止します。詳しい手順は [tools/checkerboard_calibration.md](tools/checkerboard_calibration.md) を参照してください。

`calibrate_checkerboard_extrinsics.py` は、各画像ペアから求めた変換の多数派だけを自動採用します。`checkerboard_result.json` の `pairs_rejected_outliers` や各 `pair_details[].included_in_final_estimate` を見ると、どの画像ペアが final estimate から外れたかを確認できます。外れ値が 0 でも `translation_std_mm` や後段の `aux_body_match_err_mm` が大きい場合は、撮影条件や corner order の見直しが必要です。`config_out` を `track_config_2.json` にして実行した場合は、評価用や capture 用を含む関連 `track_config_2*.json` にも同じ rig 値を同期します。

## 統合座標の評価

融合後の座標がどの程度そろっているかを確認したいときは、`track_config_2_bt_cuda_lite_dual_eval.json` を使って `track_test_cpp_2cam` を起動します。

- `enable_aux_body_tracking = true`
- `enable_fusion_trace_csv = true`
- `fusion_trace_csv_path = "fusion_eval/fusion_trace.csv"`

この config では、実行中の各フレームについて `base` 座標、`aux` の body tracking 座標、`aux` depth 補正座標、最終的な fused 座標を CSV に保存します。

`build_2cam_x64_dualverify\track_test_cpp_2cam.exe` のような 2 カメラ用ビルド出力にある exe でも確認できますが、チェッカーボード反映後の座標統合を見たい場合は、既定の `track_config_2.json` ではなく `track_config_2_bt_cuda_lite_dual_eval.json` を明示して起動してください。viewer を目視するだけでは「補助カメラの座標変換がずれている」のか「aux capture が stall して最後のフレームを表示し続けている」のかを区別できません。

例:

```bat
build_2cam_x64\track_test_cpp_2cam.exe track_config_2_bt_cuda_lite_dual_eval.json
```

保存された CSV は `tools/evaluate_fused_coordinates.py` で評価できます。

内部整合性だけを見る例:

```bat
python .\tools\evaluate_fused_coordinates.py --trace-csv .\fusion_eval\fusion_trace.csv
```

既知の固定座標と比べる例:

```bat
python .\tools\evaluate_fused_coordinates.py --trace-csv .\fusion_eval\fusion_trace.csv --static-target-mm 0 0 1500
```

外部の基準 CSV と時刻合わせして比べる例:

```bat
python .\tools\evaluate_fused_coordinates.py --trace-csv .\fusion_eval\fusion_trace.csv --reference-csv .\fusion_eval\reference.csv --reference-time-col timestamp_us --reference-x-col x_mm --reference-y-col y_mm --reference-z-col z_mm
```

`reference.csv` は自動生成されません。別系統の基準計測がある場合に、その時系列座標を手元で用意して渡します。現時点では未用意でも、内部整合性の確認までは進められます。

チェッカーボード回転の治具評価をしたい場合は、`tools/evaluate_checkerboard_fusion.py` を使います。これは `評価手法/kinect深度座標精度の評価手法（補足）.pdf` の方針に合わせて、既知角で回したチェッカーボード画像ペアから理論深度変化量を作り、`base` / `aux_transformed` / `fused` を比較するスクリプトです。

必要な入力:

- `base` / `aux_color` のチェッカーボード画像ディレクトリ
- `tools/checkerboard_rig_live.json` のような rig calibration JSON
- `track_config_2.json` のような `aux_translation_mm` / `rotation_matrix` を持つ JSON
- 各画像 stem と角度を対応付けた CSV

テンプレート JSON を使う例:

```powershell
python .\tools\evaluate_checkerboard_fusion.py --evaluation-config .\tools\checkerboard_fusion_eval_config_template.json
```

角度 CSV を直接渡す例:

```powershell
python .\tools\evaluate_checkerboard_fusion.py --base-dir .\calibration_images\SESSION_NAME\base --aux-dir .\calibration_images\SESSION_NAME\aux_color --rig-calibration .\tools\checkerboard_rig_live.json --transform-config .\track_config_2.json --angles-csv .\calibration_images\SESSION_NAME\angles.csv --board-cols 9 --board-rows 6 --square-size-mm 32 --aux-corner-order reverse --reference-stem 0001 --rotation-axis board_y --output-json .\calibration_images\SESSION_NAME\checkerboard_fusion_eval.json
```

主に見る項目:

- `overall.base.z_change_error_mm`
  - `base` 単体で見た理論深度変化とのずれ
- `overall.aux_transformed.z_change_error_mm`
  - `aux` を `base` 座標系へ移した後の深度変化誤差
- `overall.fused.z_change_error_mm`
  - 統合後の深度変化誤差
- `overall.fused_minus_base`
  - `base` に対して `fused` がどれだけ改善したか

`fused` は `z` に `aux_transformed.z` をそのまま使うため、深度だけを見る指標は `aux_transformed` と同じ値になります。差が出るのは `x, y` を含めた 3D 誤差です。詳しい手順は [tools/checkerboard_fusion_evaluation.md](tools/checkerboard_fusion_evaluation.md) を参照してください。

### 校正から回転評価までの最短フロー

校正用と評価用を分ける場合は、次の順が最も分かりやすいです。

1. 校正用セッションを撮影する。`track_config_2_capture_images.json` で複数姿勢のチェッカーボード画像を保存する。
2. `tools/checkerboard_calibration_config_template.json` の `base_dir` / `aux_dir` / `output_json` を校正用セッションへ向ける。
3. `build_2cam_x64\export_kinect_rig_calibration.exe --output .\tools\checkerboard_rig_live.json` を実行する。
4. `python .\tools\calibrate_checkerboard_extrinsics.py --calibration-config .\tools\checkerboard_calibration_config_template.json` を実行し、`track_config_2.json` へ `aux_translation_mm` / `rotation_matrix` を反映する。
5. 評価用セッションを別で撮影する。回転治具で既知角ごとに保存し、`angles.csv` を作る。
6. `tools/checkerboard_fusion_eval_config_template.json` の `base_dir` / `aux_dir` / `angles_csv` / `output_json` を評価用セッションへ向け、`square_size_mm` と corner order を実験条件に合わせる。
7. `python .\tools\evaluate_checkerboard_fusion.py --evaluation-config .\tools\checkerboard_fusion_eval_config_template.json` を実行する。
8. `checkerboard_fusion_eval.json` の `overall.fused_minus_base` を見て、`z_change_rmse_mm_delta` や `absolute_3d_rmse_mm_delta` が負になるか確認する。

このフローは「校正に使った画像でそのまま評価しない」ため、座標統合の良し悪しを切り分けやすいです。さらに runtime の実人物テストをするときは、その後に `track_config_2_bt_cuda_lite_dual_eval.json` で `fusion_trace.csv` を取り、`evaluate_fused_coordinates.py` を回します。

実測例は [tools/checkerboard_fusion_eval_report_20260626_115213_322.md](tools/checkerboard_fusion_eval_report_20260626_115213_322.md) にまとめています。

主に見る項目:

- `cross_camera_alignment.base_vs_aux_body`
  - `base` と `aux` body tracking の 3D 一致度
- `cross_camera_alignment.base_vs_aux_depth`
  - `base` と `aux` depth 補正点の 3D 一致度
- `z_delta_mm`
  - fused の Z が `base` 単体よりどれだけ動いたか
- `reference_error` / `static_target_error`
  - 真値がある場合の絶対誤差。`base` と `fused` の両方、および `fused_minus_base` の改善量を見られます

CSV の生データでは、次の列がチェッカーボード反映後の確認に有用です。

- `aux_body_used`
  - `aux` body tracking を fused 座標へ実際に使ったか。`0` が続く場合は `aux` 側が外れすぎて match gate に落ちている可能性があります
- `aux_body_match_err_mm`
  - `aux` body tracking を `base` 座標系へ変換した点と `base` body tracking の差。大きな値が続く場合は、チェッカーボード外部パラメータの向き違い、カメラ serial の取り違え、または人物追跡の不一致を疑ってください
- `aux_match_err_mm`
  - `aux_depth` 補正点の探索誤差。`aux_body_match_err_mm` と合わせて見ると、どこでずれているかを切り分けやすくなります

`max_aux_body_match_error_mm` を超えた `aux` body tracking 点は fused 座標へ使わず、`aux_body_used=0` になります。チェッカーボード反映後に `source=base_xy + aux_body_z` が出ていても、`aux_body_match_err_mm` が大きいままなら外部パラメータを再確認してください。

真の精度評価には、固定治具などで既知の 3D 座標を用意するか、別系統の基準計測 CSV を与える必要があります。基準なしでも、2台の一致度やフレーム間のばらつきから外部キャリブレーションの良し悪しはかなり見えます。

現時点の実用上の判定基準は次のとおりです。

- `座標統合が動いている` とみなす条件
  - aux viewer 上で黄色ポインタと緑ポインタが耳の近くで重なって見える
  - `source=base_xy + aux_body_z` または `source=base_xy + aux_depth_z` が継続的に出る
  - `aux_body_match_err_mm` と `aux_match_err_mm` が数十 mm から 100 mm 前後で収まる
- `main Kinect の深さ方向補正を証明できた` とみなす条件
  - `--static-target-mm` または `--reference-csv` で真値を与える
  - `static_target_error` または `reference_error` の `fused_minus_base` で、`z_axis_mae_mm_delta` / `z_axis_rmse_mm_delta` が負になる
  - 可能なら `euclidean_mae_mm_delta` / `euclidean_rmse_mm_delta` も負になる

viewer のポインタ色は次の意味です。

- aux viewer の黄色
  - `base` 側で見えた耳位置を、現在の外部パラメータで aux depth 画像へ写した予測位置
- aux viewer の緑
  - aux 側で実際に使えた位置。aux body tracking の耳があればそれを、なければ aux depth でサンプリングできた点を示す
- base viewer の緑
  - 最終的な fused ear を base color 画像へ投影した位置

## 既知の注意点

- `CUDA provider probe failed ... error 1114` が出ても、その後に `Base body tracker created with mode: gpu_cuda` が出る場合は実運用上 tracker 作成に成功している
- `subordinate_delay_off_master_usec = 160` は subordinate の color capture タイミング設定であり、ログ上の `phase_delta_us` と 1:1 に一致する値ではない
- `raw_delta_us` はフレーム周期の整数倍だけずれて見えることがあるので、同期確認では `phase_delta_us` / `phase_error_us` を優先して見る
- 融合用の `aux_translation_mm` / `rotation_matrix` はチェッカーボード推定値へ更新済みだが、実人物の真値基準 `reference.csv` はまだ無いため、絶対精度は未証明
- `aux` stream stall 時は再起動と tracker 再生成を試みるが、USB や電源条件が悪いと再起動に失敗する可能性は残る
- `aux` viewer が固まって見えるときは、実際には `aux capture stalled; viewer is showing the last received depth frame.` の状態で最後のフレームを再描画している場合がある。まず `track_test_2_runtime.log` の `Aux Kinect capture stopped updating...` と再起動ログを確認する

## 今後の確認事項

- 既知角チェッカーボード評価で、実験ごとの `angles.csv` と結果 JSON を蓄積し、`fused_minus_base` の傾向を見やすくする
- 可能になった段階で `reference.csv` を用いた `base` / `fused` の絶対誤差比較
- 実人物を入れた状態で `aux_body=FOUND` と `source=base_xy + aux_body_z` が安定して出るか
- `aux_body_z` と `aux_depth_z` のどちらが実運用で安定するか
- `sample_aux_depth_point_for_base_joint()` の探索条件の最適化
- 長時間運転時の `aux` 再起動経路の安定性

## 退避済みファイル

現在のチェッカーボード撮影・座標統合検証フローでは使っていない診断用 config と補助スクリプトは、`archived_unused_configs_and_scripts/` へ移しています。

- 例: `measure_vram_bodytracking.ps1`
- 例: `track_config_2_bt_cuda*.json`
- 例: `track_config_2_capture_only.json`

将来ふたたび使う場合は、この退避フォルダから root へ戻すか、必要に応じて参照パスを直接指定してください。

## 変更履歴

重要な変更は、コミット単位で次の表に残します。

| Date | Commit | Summary |
| --- | --- | --- |
| 2026-06-26 | `7af2675` | 既知角で回したチェッカーボード画像から、`base` / `aux_transformed` / `fused` の深さ方向変化を比較する `evaluate_checkerboard_fusion.py` と関連ドキュメントを追加。 |
| 2026-05-13 | `4a32b68` | 現行フローで使わない診断用 config と補助スクリプトを `archived_unused_configs_and_scripts/` へ退避し、root を最小構成へ整理。 |
| 2026-05-13 | `625958d` | README を現状に合わせて更新し、キャリブレーション・評価・注意点の整理を反映。 |
| 2026-05-13 | `98f6ac8` | `evaluate_fused_coordinates.py` が `fused` だけでなく `base` も真値比較できるよう拡張し、改善量を直接比較可能にした。 |
| 2026-05-13 | `da88f7f` | `track_config_2.json` 更新時に、関連する `track_config_2*.json` へ rig 値を自動同期するよう修正。 |
| 2026-05-13 | `358789c` | チェッカーボード外部パラメータ推定に外れ値除外を追加し、180 度別解や誤検出の混入に強くした。 |
| 2026-05-13 | `684f7ff` | 接続中の Azure Kinect 実機から rig calibration JSON を書き出す `export_kinect_rig_calibration.cpp` を追加。 |
| 2026-05-13 | `338ff7e` | 未初期化の `checkerboard_rig_template.json` を誤って本番入力に使った場合に、キャリブレーションを停止する安全策を追加。 |
| 2026-04-29 | `d5bfb41` | `fusion_trace.csv` の保存と `evaluate_fused_coordinates.py` による内部整合性評価フローを追加。 |
| 2026-04-29 | `d39d41a` | `track_test_2.cpp` にチェッカーボード画像キャプチャ機能を追加し、`base` / `aux_color` の画像ペア保存を可能にした。 |
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
