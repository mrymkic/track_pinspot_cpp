# Codex 依頼仕様書: チェッカーボード回転評価治具

## 目的

Azure Kinect 2台を用いた統合座標，特に `base_xy + aux_z` による統合深さ座標の評価に使うため，チェッカーボードを既知角度で回転・固定できる治具を作成する。

この治具は，チェッカーボードの中央列を通る鉛直軸を回転軸として，ボードを左右方向に回転させることを目的とする。回転時の手作業誤差をできるだけ減らし，角度ごとの深さ変化量を安定して取得できる構造にする。

## 管理方針

本リポジトリ `track_pinspot_cpp` 内で，評価用ハードウェア治具として管理する。

以下のディレクトリを新規作成すること。

```text
hardware/checkerboard_rotation_jig/
  README.md
  openscad/
    checkerboard_rotation_jig.scad
    params.scad
    parts/
      mic_stand_mount.scad
      vertical_axis_frame.scad
      shaft_and_bearing.scad
      board_clamp.scad
      angle_lock_plate.scad
  export_stl.ps1
```

STLファイルは生成物なので，原則としてリポジトリには追加しない。必要であれば `hardware/checkerboard_rotation_jig/stl/` を出力先とし，`.gitignore` に追加する。

## 使用CAD

OpenSCAD を用いてパラメトリックに設計する。

理由は以下である。

- 主要部品がブラケット，軸受け，クランプ，角度固定板などの機械部品であること
- 寸法変更が多く発生すること
- Gitで差分管理しやすいこと
- Codexで修正しやすいこと

CadQuery は今回は不要。OpenSCADのみで完結させる。

## 設計対象

可能な限り3Dプリンタで作成する。ただし，実用上必要な締結部品として，M3/M4/M5/M6程度の市販ボルト，ナット，ワッシャは使用してよい。

3Dプリントする主部品は以下とする。

1. マイクスタンド接続部
2. 縦軸支持フレーム
3. 上側軸受け
4. 下側軸受け
5. 回転軸，または回転軸を保持するクランプ
6. チェッカーボード固定クランプ
7. 角度固定プレート
8. 角度固定ピン・ノブ用部品

## 想定設備

現状の設備は以下である。

- チェッカーボードを貼り付けた段ボール
- マイクスタンド
- マイクスタンド用アタッチメント
  - 3/8インチ
  - 5/8インチ

ただし，段ボールはたわみやすいため，治具側ではボードを補強できる設計にする。具体的には，チェッカーボードを貼った板を左右または上下から挟み込めるクランプ構造にする。

## 基本構造

### 全体構造

マイクスタンド上に，チェッカーボードを鉛直軸回りに回転できるフレームを取り付ける。

概念図:

```text
マイクスタンド
    │
    ▼
[マイクスタンド接続部]
    │
    ▼
[縦軸支持フレーム]
    │
    ├── 上側軸受け
    │
    ├── 回転軸
    │
    ├── 下側軸受け
    │
    └── 角度固定プレート
            │
            ▼
      [ボード固定クランプ]
            │
            ▼
      チェッカーボード
```

### 回転軸

普通の蝶番ではなく，縦シャフト方式を基本とする。

要件:

- 回転軸はチェッカーボード中央列に一致させる。
- 軸は鉛直方向に配置する。
- 軸の上下を軸受けで支える。
- 軸中心線が評価時の回転基準になるようにする。
- 3Dプリント軸の場合は，直径を太めにする。

初期値:

```text
shaft_diameter_mm = 12
shaft_clearance_mm = 0.4
shaft_length_mm = board_height_mm + 40
```

軸径とクリアランスは `params.scad` で変更可能にする。

### 軸受け

上下2点支持とする。

要件:

- 軸受け穴径 = `shaft_diameter_mm + shaft_clearance_mm`
- 軸の抜け止め構造を付ける。
- 軸受け部には補強リブを付ける。
- 上側・下側軸受けの間隔を広く取り，回転軸の傾きを抑える。

### ボード固定クランプ

チェッカーボードまたはその支持板を固定する部品を作る。

要件:

- ボード寸法をパラメータ化する。
- ボード厚さをパラメータ化する。
- 中央列位置を `board_axis_offset_x_mm` として調整可能にする。
- ボードを挟むクランプ構造にする。
- クランプはボードの裏面に取り付ける想定とする。
- 必要に応じて，ボードの反りを抑える背面補強バーを追加できる設計にする。

初期値:

```text
board_width_mm = 297
board_height_mm = 210
board_thickness_mm = 5
board_axis_offset_x_mm = 0
```

A4横向き程度を初期値とするが，必ず変更可能にする。

### マイクスタンド接続部

マイクスタンドに取り付けるためのベース部品を作る。

要件:

- 3/8インチ，5/8インチのアタッチメントに対応できる余地を残す。
- ねじ形状を3Dプリントで完全再現するのではなく，初期設計では貫通穴，締め込み，または既存アタッチメントを挟み込む構造を優先する。
- 将来的にねじ形状を追加できるよう，寸法パラメータは分離しておく。
- マイクスタンド上で回転しないよう，クランプまたはロックねじ用の構造を用意する。

初期案:

- 中央に `stand_adapter_outer_diameter_mm` の円筒アタッチメントを受ける穴を設ける。
- 横方向からM4またはM5ねじで締める割り締めクランプ構造にする。
- 3/8インチ・5/8インチ変換アタッチメントを使用する場合も固定できるようにする。

初期値:

```text
stand_adapter_outer_diameter_mm = 18
stand_clamp_clearance_mm = 0.5
stand_clamp_bolt_diameter_mm = 5
```

実寸が不明なため，READMEに「実測して変更すること」と明記する。

### 角度固定機構

ボードを所定角度で固定できる角度固定プレートを作る。

要件:

- 回転範囲は少なくとも -30° から +30° とする。
- 10°刻みの固定穴を設ける。
- 0°，±10°，±20°，±30°を初期設定にする。
- 固定穴の角度リストはパラメータ化する。
- 角度固定はピンまたはノブで行う。
- 角度固定機構にはガタが出るため，READMEに「実角度はデジタル角度計または画像推定で確認する」と書く。

初期値:

```text
angle_list_deg = [-30, -20, -10, 0, 10, 20, 30]
angle_plate_radius_mm = 80
angle_pin_diameter_mm = 5
angle_pin_clearance_mm = 0.3
```

### 補強

要件:

- 垂直支持フレームには三角リブを付ける。
- マイクスタンド接続部から軸受けまでの片持ち長さを短くする。
- ボード重量によるたわみを抑える。
- 印刷向きを考慮し，過度なサポートが不要な形状を優先する。

## パラメータ設計

`openscad/params.scad` に主要寸法を集約する。

最低限，以下のパラメータを定義すること。

```scad
// Board
board_width_mm = 297;
board_height_mm = 210;
board_thickness_mm = 5;
board_axis_offset_x_mm = 0;

// Shaft
shaft_diameter_mm = 12;
shaft_clearance_mm = 0.4;
shaft_length_mm = board_height_mm + 40;

// Bearing
bearing_block_width_mm = 36;
bearing_block_depth_mm = 28;
bearing_block_height_mm = 24;

// Stand mount
stand_adapter_outer_diameter_mm = 18;
stand_clamp_clearance_mm = 0.5;
stand_clamp_bolt_diameter_mm = 5;

// Angle lock
angle_list_deg = [-30, -20, -10, 0, 10, 20, 30];
angle_plate_radius_mm = 80;
angle_pin_diameter_mm = 5;
angle_pin_clearance_mm = 0.3;

// Fasteners
m3_clearance_mm = 3.4;
m4_clearance_mm = 4.5;
m5_clearance_mm = 5.5;
m6_clearance_mm = 6.6;

// Print tolerance
general_clearance_mm = 0.3;
```

## OpenSCAD実装要件

### checkerboard_rotation_jig.scad

全体アセンブリを表示するトップファイルにする。

要件:

- `include <params.scad>` を使う。
- 各部品ファイルを `use` または `include` で読み込む。
- `assembly()` を定義する。
- `part` 変数で個別部品を切り替えられるようにする。

例:

```scad
part = "assembly"; // assembly, stand_mount, frame, shaft, board_clamp, angle_lock_plate

if (part == "assembly") assembly();
else if (part == "stand_mount") mic_stand_mount();
else if (part == "frame") vertical_axis_frame();
else if (part == "shaft") shaft();
else if (part == "board_clamp") board_clamp();
else if (part == "angle_lock_plate") angle_lock_plate();
```

### 各部品ファイル

各ファイルには，単独で呼び出せるmoduleを定義する。

- `mic_stand_mount()`
- `vertical_axis_frame()`
- `shaft()`
- `bearing_block_top()`
- `bearing_block_bottom()`
- `board_clamp()`
- `angle_lock_plate()`
- `angle_lock_pin_knob()`

## export_stl.ps1

OpenSCAD CLIで主要部品をSTL出力するPowerShellスクリプトを作る。

出力先:

```text
hardware/checkerboard_rotation_jig/stl/
```

例:

```powershell
openscad -o stl/stand_mount.stl -D 'part="stand_mount"' openscad/checkerboard_rotation_jig.scad
openscad -o stl/frame.stl -D 'part="frame"' openscad/checkerboard_rotation_jig.scad
openscad -o stl/shaft.stl -D 'part="shaft"' openscad/checkerboard_rotation_jig.scad
openscad -o stl/board_clamp.stl -D 'part="board_clamp"' openscad/checkerboard_rotation_jig.scad
openscad -o stl/angle_lock_plate.stl -D 'part="angle_lock_plate"' openscad/checkerboard_rotation_jig.scad
```

## README.md 要件

`hardware/checkerboard_rotation_jig/README.md` には以下を書く。

1. 目的
2. 全体構成
3. 必要部品
4. 寸法の測り方
5. パラメータ変更方法
6. STL出力方法
7. 印刷時の注意
8. 組み立て手順
9. 実験時の使い方
10. 評価上の注意

### 評価上の注意として必ず書くこと

- 角度固定穴の角度は設計値であり，実角度とはずれる可能性がある。
- 実験ではデジタル角度計，分度器，または画像から推定した角度を記録する。
- 回転軸がチェッカーボード中央列からずれると，理論深さ変化量に誤差が入る。
- 3Dプリント品にはガタがあるため，中央列の実測深さを各角度で記録する。
- 段ボールはたわむため，可能であれば硬い板にチェッカーボードを貼る。

## 実験との接続

この治具は，以下のような評価に使用する。

1. チェッカーボードを0°に固定する。
2. Kinect座標系で各格子点の統合座標を取得する。
3. 角度固定プレートにより，ボードを ±10°，±20°，±30° に固定する。
4. 各角度で統合座標を取得する。
5. 中央列を基準として，各列の深さ変化量を算出する。
6. 幾何的に計算した理論深さ変化量と比較する。

ただし，評価時は治具の誤差を完全には排除できないため，以下も記録する。

- 設計角度
- 実測角度
- 中央列の平均深さ
- 中央列の標準偏差
- ボードのたわみや固定状態

## 受け入れ条件

Codexは以下を満たす変更を提出すること。

- `hardware/checkerboard_rotation_jig/` 以下にOpenSCADモデルとREADMEが追加されている。
- `params.scad` に主要寸法が集約されている。
- `checkerboard_rotation_jig.scad` で全体アセンブリを確認できる。
- `part` 変数により主要部品を個別STL出力できる。
- `export_stl.ps1` で主要部品をSTLに出力できる。
- READMEに組み立て手順と評価上の注意が書かれている。
- STLなどの生成物はコミットしない。
- 既存のC++トラッキングコードには影響を与えない。

## Codexへの依頼文

以下の内容で実装してください。

> `track_pinspot_cpp` リポジトリ内に，Azure Kinectの統合深さ座標評価に使用するチェッカーボード回転治具のOpenSCADモデルを追加してください。  
> 目的は，チェッカーボード中央列を通る鉛直軸を回転軸として，ボードを -30° から +30° 程度まで固定角度で回転させられる治具を作ることです。  
> 3Dプリント部品を中心に構成し，マイクスタンドに固定できるベース，縦軸支持フレーム，上下軸受け，回転軸，ボード固定クランプ，角度固定プレートを作成してください。  
> 主要寸法は `params.scad` にまとめ，ボード寸法，ボード厚，軸径，軸クリアランス，マイクスタンドアタッチメント径，角度固定穴の角度リストを変更可能にしてください。  
> STLはコミットせず，OpenSCADソースとSTL出力用PowerShellスクリプトのみ追加してください。  
> READMEには，目的，全体構成，必要部品，寸法の測り方，STL出力方法，印刷時の注意，組み立て手順，実験時の使い方，評価上の注意を書いてください。  
> 既存のC++コードには変更を加えないでください。
