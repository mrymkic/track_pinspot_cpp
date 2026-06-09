# Checkerboard Rotation Jig

## 目的

Azure Kinect の統合深さ座標評価で使うために，チェッカーボードを鉛直軸まわりに既知角度で回転・固定する治具です。  
このモデルは OpenSCAD で寸法変更しやすいように構成してあり，マイクスタンド上に載せる前提で，縦シャフト方式の回転軸，軸受けフレーム，ボード固定クランプ，角度固定プレートをまとめています。

## 全体構成

- `openscad/checkerboard_rotation_jig.scad`
  - 全体アセンブリ表示用のトップファイルです。
- `openscad/params.scad`
  - 主要寸法を集約したパラメータファイルです。
- `openscad/parts/mic_stand_mount.scad`
  - 3/8 インチ系アタッチメントを既存変換アダプタごと割り締めするベースです。
- `openscad/parts/vertical_axis_frame.scad`
  - マイクスタンド上に載る縦軸支持フレームです。
- `openscad/parts/shaft_and_bearing.scad`
  - 回転シャフト，ストップカラー，上下軸受けブロックです。
- `openscad/parts/board_clamp.scad`
  - ボード裏面側のキャリアと，前面から押さえるクランプバーです。
- `openscad/parts/angle_lock_plate.scad`
  - 角度固定プレートと固定ピン用ノブです。

## 必要部品

- 3D プリント部品
  - `stand_mount`
  - `frame`
  - `shaft`
  - `board_clamp`
  - `angle_lock_plate`
- 市販部品の目安
  - M5 ボルト 4 本程度
  - M4 ボルト 6 本以上
  - M3 ボルト 2 本程度
  - 対応するナット，ワッシャ
  - 3/8 インチまたは 5/8 インチ変換アタッチメント
  - チェッカーボードを貼った板または段ボール

## 寸法の測り方

- マイクスタンドアタッチメント外径
  - ノギスで実測し，`stand_adapter_outer_diameter_mm` に反映してください。
- ボード幅，高さ，厚さ
  - 実際に使う板の外形を測り，`board_width_mm`，`board_height_mm`，`board_thickness_mm` を更新してください。
- 回転軸位置の左右ずれ
  - チェッカーボード中央列とクランプ中心がずれる場合は `board_axis_offset_x_mm` で補正してください。
- 軸まわり
  - 使用するシャフト径に合わせて `shaft_diameter_mm` を変更し，組み付け具合に応じて `shaft_clearance_mm` を微調整してください。

## パラメータ変更方法

`openscad/params.scad` を編集します。最低限，次の項目を変更できるようにしてあります。

- `board_width_mm`
- `board_height_mm`
- `board_thickness_mm`
- `board_axis_offset_x_mm`
- `shaft_diameter_mm`
- `shaft_clearance_mm`
- `stand_adapter_outer_diameter_mm`
- `angle_list_deg`

`openscad/checkerboard_rotation_jig.scad` では `part` を切り替えることで，アセンブリ表示または主要部品の個別出力ができます。

```scad
part = "assembly"; // assembly, stand_mount, frame, shaft, board_clamp, angle_lock_plate
```

## STL出力方法

PowerShell から `hardware/checkerboard_rotation_jig` に移動して実行します。

```powershell
./export_stl.ps1
```

`openscad.exe` が PATH に無い場合は，実行ファイルを指定してください。

```powershell
./export_stl.ps1 -OpenSCADPath "C:\Program Files\OpenSCAD\openscad.exe"
```

生成先は `hardware/checkerboard_rotation_jig/stl/` です。STL は生成物なのでコミットしません。

## 印刷時の注意

- フレームは大きいので，プリンタの造形サイズに収まるか先に確認してください。
- 回転シャフトと軸受けは積層方向で強度が変わるため，必要なら外周数とインフィル率を上げてください。
- クランプ部とストップカラーは締め付けで割れやすいので，過度に締めすぎないでください。
- 寸法誤差が大きいプリンタでは `general_clearance_mm` と `shaft_clearance_mm` を増やしてください。
- クランプバーや角度固定プレートは，必要に応じてサポートなしで置ける向きにスライサで回転してください。

## 組み立て手順

1. `stand_mount` をマイクスタンドアタッチメントまたは変換アダプタに差し込み，側面ボルトで割り締めします。
2. `frame` を `stand_mount` 上に置き，フット部の穴を M5 ボルトで固定します。
3. 上下の軸受けブロックをフレームのアーム上に載せ，M5 ボルトで固定します。
4. `shaft` を上下軸受けに通し，ストップカラーを上下端側で固定して抜け止めにします。
5. `board_clamp` のキャリアをシャフト中央付近に取り付け，クランプバーでボードを前後から挟みます。
6. `angle_lock_plate` を下側に配置し，シャフトへ固定します。
7. 固定ピンをフレーム側ガイドと角度固定穴に差し込み，所定角度でロックします。

## 実験時の使い方

1. `assembly` 表示で干渉と寸法感を確認します。
2. 実機ではボードを 0° に合わせて固定し，中央列が回転軸に近いことを確認します。
3. Azure Kinect 側で基準姿勢の統合座標を取得します。
4. `angle_list_deg` に対応する穴でボードを固定し，各角度で統合座標を取得します。
5. 中央列を基準に各列の深さ変化を比較し，理論値との差を評価します。

## 評価上の注意

- 角度固定穴の角度は設計値であり，実角度とはずれる可能性がある。
- 実験時にはデジタル角度計，分度器，または画像推定により実角度を確認する。
- 回転軸がチェッカーボード中央列からずれると，理論深さ変化量に誤差が入る。
- 3Dプリント品にはガタがあるため，中央列の実測深さを各角度で記録する。
- 段ボールはたわむため，可能であれば硬い板にチェッカーボードを貼る。

追加で次の記録を残すと比較しやすくなります。

- 設計角度
- 実測角度
- 中央列の平均深さ
- 中央列の標準偏差
- ボードの反りや固定状態
