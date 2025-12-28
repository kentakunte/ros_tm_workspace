# TM Robot ROS2 Workspace

TM Robot（Techman Robot）用のROS2ワークスペースです。MoveItによるモーションプランニング、Omniverse Isaac Simとの統合、およびピック＆プレースデモを含みます。

## 📦 パッケージ構成

### 1. tmr_ros2
TM Robotの公式ROS2ドライバーとMoveIt設定

- **tm_driver**: TM Robot通信ドライバー
- **tm_description**: URDFモデル定義
- **tm_msgs**: カスタムメッセージ・サービス定義
- **tm5-700_moveit_config**: TM5-700用MoveIt設定（ピック＆プレースデモ付き）
- **tm5-900_moveit_config**: TM5-900用MoveIt設定
- **tm12_moveit_config**: TM12用MoveIt設定
- **tm14_moveit_config**: TM14用MoveIt設定
- その他のバリエーション（tm12x, tm14x, tm5x-700, tm5x-900）

### 2. tm_ros2_isaac_example
NVIDIA Omniverse Isaac Simとの統合例

- USD形式のロボットモデル
- Action Graphサンプル
- デモワールド

## 🚀 クイックスタート

### 前提条件

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# MoveIt
sudo apt install ros-humble-moveit

# 依存パッケージ
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
```

### ビルド

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 📖 主要機能

### 1. MoveItでのロボット制御（RViz）

```bash
# TM5-700の例
ros2 launch tm5-700_moveit_config demo.launch.py
```

RVizが起動し、インタラクティブマーカーで動作計画・実行が可能です。

**機能**：
- ✅ インタラクティブマーカーによる目標姿勢設定
- ✅ OMPLプランナーによる軌道計画
- ✅ RVizでの可視化
- ✅ 衝突回避

### 2. ピック＆プレースデモ

A地点のボックスをB地点に運ぶデモプログラム：

```bash
# デモ実行
ros2 run tm5-700_moveit_config pick_and_place_simple.py
```

詳細は [script/README_PICK_AND_PLACE.md](src/tmr_ros2/tm5-700_moveit_config/script/README_PICK_AND_PLACE.md) を参照。

### 3. トラブルシューティング

インタラクティブマーカーが正しく動作しない場合は、[TROUBLESHOOTING.md](src/tmr_ros2/tm5-700_moveit_config/TROUBLESHOOTING.md) を参照してください。

## 📁 ディレクトリ構造

```
ros2_ws/
├── src/
│   ├── tm_ros2_isaac_example/  # Isaac Sim統合
│   │   ├── assets/             # USDモデル
│   │   └── tm5s_demo/          # デモスクリプト
│   └── tmr_ros2/               # TMロボット本体
│       ├── tm_driver/          # 通信ドライバー
│       ├── tm_description/     # URDFモデル
│       ├── tm_msgs/            # メッセージ定義
│       └── tm5-700_moveit_config/  # MoveIt設定
│           ├── config/         # 設定ファイル
│           ├── launch/         # 起動ファイル
│           └── script/         # Pythonスクリプト
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
