# TM Robot ROS2 ワークスペース

TM Robot (Techman Robot) の ROS2 ワークスペース。MoveIt モーションプランニング、Omniverse Isaac Sim 統合、ビジョンベースのオブジェクトトラッキング機能を搭載。

## 📦 パッケージ概要

### 1. tmr_ros2
TM Robot の公式 ROS2 ドライバーと MoveIt 設定

- **tm_driver**: TM Robot 通信ドライバー
- **tm_description**: URDF モデル定義
- **tm_msgs**: カスタムメッセージ/サービス定義
- **tm5-700_moveit_config**: TM5-700 用 MoveIt 設定（ピック＆プレースデモ付き）
- **tm5-900_moveit_config**: TM5-900 用 MoveIt 設定
- **tm12_moveit_config**: TM12 用 MoveIt 設定
- **tm14_moveit_config**: TM14 用 MoveIt 設定
- その他のバリエーション（tm12x, tm14x, tm5x-700, tm5x-900）

### 2. tm_ros2_isaac_example
NVIDIA Omniverse Isaac Sim との統合サンプル

- USD 形式のロボットモデル
- Action Graph サンプル
- デモワールド

### 3. ビジョンベースオブジェクトトラッキング
赤色オブジェクト検出・追跡用のコンピュータビジョンスクリプト

- **color_tracker.py**: 基本的な色ベースオブジェクト検出
- **object_to_base_node.py**: MoveIt 統合オブジェクト検出
- **joint_bridge.py**: Isaac Sim 用ジョイント状態ブリッジ

## 🚀 クイックスタート

### 前提条件

```bash
# ROS2 Jazzy (Ubuntu Noble)
sudo apt install ros-jazzy-desktop

# MoveIt2
sudo apt install ros-jazzy-moveit

# 依存関係
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-moveit-py
sudo apt install python3-opencv
sudo apt install ros-jazzy-cv-bridge
```

### ビルド

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 📖 主要機能

### 1. MoveIt によるロボット制御（RViz）

```bash
# TM5-700 の例（Isaac Sim 使用時）
ros2 launch tm5-700_moveit_config demo.launch.py use_sim_time:=true
```

RViz が起動し、インタラクティブマーカーで動作計画・実行が可能です。

**機能**：
- ✅ インタラクティブマーカーによる目標姿勢設定
- ✅ OMPL プランナーによる軌道計画
- ✅ RViz での可視化
- ✅ 衝突回避

### 2. ビジョンベースオブジェクトトラッキング

コンピュータビジョンを使用して赤色オブジェクトを検出・追跡し、自動到達可能性チェックとモーションプランニングを実行します。

#### セットアップ
実行前に conda を無効化してください：
```bash
conda deactivate
```

#### カラートラッカー（検出のみ）
```bash
./run_color_tracker.sh
```
カメラ画像内の赤色オブジェクトを検出し、位置を表示します。

#### MoveIt 統合オブジェクトトラッカー
```bash
./run_object_tracker.sh
```

**機能**：
- 🎯 HSV カラーフィルタリングによる赤色オブジェクト検出
- 📐 カメラ座標系からベース座標系への TF 変換
- ✅ ワークスペース到達可能性チェック（距離 0.15-1.5m、高さ -0.5～1.5m）
- 🤖 検出オブジェクトの 20cm 上への自動モーションプランニング
- 🔄 動作間の 3 秒クールダウン

**必要条件**：
- TM5-700 ロボットとカメラを使用した Isaac Sim の実行
- MoveIt デモの実行：`ros2 launch tm5-700_moveit_config demo.launch.py use_sim_time:=true`
- `/camera/image_raw` へのカメラパブリッシュ
- TF フレーム：`base` と `eih_2d_camera`

**出力例**：
```
============================================================
RED BOX DETECTED
Position in world frame: x=-0.758m, y=0.060m, z=0.217m
Distance from origin: 0.760m
Height: 0.217m
------------------------------------------------------------
✅ BOX IS REACHABLE
   - Distance OK: 0.760m (range: 0.15-1.5m)
   - Height OK: 0.217m (range: -0.5-1.5m)
============================================================
📍 Moving to position 20cm above box...
Sending goal: (-0.76, 0.06, 0.42)
✓ Goal accepted, planning and executing...
✅ Motion executed successfully!
```

### 3. ピック＆プレースデモ

A 地点のボックスを B 地点に運ぶデモプログラム：

```bash
# デモ実行
ros2 run tm5-700_moveit_config pick_and_place_simple.py
```

詳細は [script/README_PICK_AND_PLACE.md](src/tmr_ros2/tm5-700_moveit_config/script/README_PICK_AND_PLACE.md) を参照してください。

### 4. トラブルシューティング

インタラクティブマーカーが正しく動作しない場合は、[TROUBLESHOOTING.md](src/tmr_ros2/tm5-700_moveit_config/TROUBLESHOOTING.md) を参照してください。

## 📁 ディレクトリ構造

```
ros2_ws/
├── color_tracker.py              # 基本的な色検出
├── object_to_base_node.py        # MoveIt 統合オブジェクトトラッキング
├── joint_bridge.py               # ジョイント状態ブリッジ
├── run_color_tracker.sh          # カラートラッカー起動スクリプト
├── run_object_tracker.sh         # オブジェクトトラッカー起動スクリプト
├── src/
│   ├── tm_ros2_isaac_example/    # Isaac Sim 統合
│   │   ├── assets/               # USD モデル
│   │   └── tm5s_demo/            # デモスクリプト
│   └── tmr_ros2/                 # TM Robot 本体
│       ├── tm_driver/            # 通信ドライバー
│       ├── tm_description/       # URDF モデル
│       ├── tm_msgs/              # メッセージ定義
│       └── tm5-700_moveit_config/  # MoveIt 設定
│           ├── config/           # 設定ファイル
│           ├── launch/           # 起動ファイル
│           └── script/           # Python スクリプト
│               ├── pick_and_place_simple.py
│               ├── pick_and_place_demo.py
│               └── moveit_diagnostics.py
├── build/
├── install/
└── log/
```

## 🎯 サポートロボット

- TM5-700 (700mm reach, 6kg payload)
- TM5-900 (900mm reach, 6kg payload)
- TM12 (1300mm reach, 12kg payload)
- TM14 (1100mm reach, 14kg payload)
- および各種バリエーション（x付きモデル）

## 🔧 開発

### 新しいロボットモデルの追加

1. `tm_description/xacro/` に URDF/Xacro を追加
2. MoveIt Setup Assistant で設定生成
3. `launch/` ファイルを作成

### カスタムピック＆プレーススクリプト

[pick_and_place_simple.py](src/tmr_ros2/tm5-700_moveit_config/script/pick_and_place_simple.py) をベースに、位置やパラメータをカスタマイズできます。

## 📚 ドキュメント

- [ピック＆プレースREADME](src/tmr_ros2/tm5-700_moveit_config/script/README_PICK_AND_PLACE.md)
- [トラブルシューティングガイド](src/tmr_ros2/tm5-700_moveit_config/TROUBLESHOOTING.md)
- [TMR ROS2 README](src/tmr_ros2/README.md)
- [Isaac Example README](src/tm_ros2_isaac_example/README.md)

## 📄 ライセンス

このプロジェクトは BSD-3-Clause ライセンスの下で公開されています。

## 🙏 謝辞

- [TechMan Robot](https://www.tm-robot.com/) - 公式ROS2ドライバー
- [MoveIt](https://moveit.ros.org/) - モーションプランニングフレームワーク
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/) - シミュレーション環境

## 🔗 関連リンク

- [TM Robot Official](https://www.tm-robot.com/)
- [MoveIt Documentation](https://moveit.ros.org/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
