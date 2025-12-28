# TM5-700 ピック＆プレースデモ

## 概要

TM5-700ロボットアームを使用して、A地点のボックスをB地点に運ぶピック＆プレース作業を可視化するデモプログラムです。

## ファイル構成

```
tm5-700_moveit_config/
├── script/
│   ├── pick_and_place_simple.py    # シンプル版（MoveIt Commander使用）
│   └── pick_and_place_demo.py      # 高度版（MoveItPy使用）
└── launch/
    └── pick_and_place.launch.py    # デモ起動用launchファイル
```

## 必要な依存パッケージ

```bash
# MoveIt関連
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-resources
sudo apt install ros-humble-moveit-configs-utils

# Python MoveIt
pip3 install moveit
```

## 実行方法

### 方法1: Launchファイルで起動（推奨）

```bash
# ビルド
cd ~/ros2_ws
colcon build --packages-select tm5-700_moveit_config

# 起動
source install/setup.bash
ros2 launch tm5-700_moveit_config pick_and_place.launch.py
```

### 方法2: 手動起動

ターミナル1: MoveIt + RVizを起動
```bash
ros2 launch tm5-700_moveit_config demo.launch.py
```

ターミナル2: デモスクリプトを実行
```bash
# シンプル版
ros2 run tm5-700_moveit_config pick_and_place_simple.py

# または高度版
ros2 run tm5-700_moveit_config pick_and_place_demo.py
```

## プログラムの動作フロー

```
1. ホームポジション
   ↓
2. ボックスをA地点に配置
   ↓
3. A地点上方へアプローチ
   ↓
4. 降下してグリップ（ボックスをアタッチ）
   ↓
5. 持ち上げ
   ↓
6. B地点上方へ移動
   ↓
7. 降下して配置（ボックスをデタッチ）
   ↓
8. 上昇
   ↓
9. ホームポジションに戻る
```

## パラメータのカスタマイズ

### 位置の変更

`pick_and_place_simple.py`の以下の部分を編集：

```python
# 位置定義
point_a = (0.4, 0.2, 0.1)    # A地点 (x, y, z) [m]
point_b = (0.4, -0.2, 0.1)   # B地点 (x, y, z) [m]
approach_height = 0.15       # アプローチ高さ [m]
grip_height = 0.08          # グリップ高さ [m]
```

### ボックスサイズの変更

```python
self.add_box(box_name, point_a, size=(0.05, 0.05, 0.05))  # (x, y, z) [m]
```

### 速度の変更

```python
self.move_group.set_max_velocity_scaling_factor(0.5)      # 0.0-1.0
self.move_group.set_max_acceleration_scaling_factor(0.5)  # 0.0-1.0
```

## プログラミングの要点

### 1. シーン管理
- `scene.add_box()`: 環境にオブジェクトを追加
- `scene.remove_world_object()`: オブジェクトを削除
- 衝突回避計算に使用

### 2. グリップのシミュレーション
- `scene.attach_box()`: ボックスをエンドエフェクタにアタッチ（把持）
- `scene.remove_attached_object()`: ボックスをデタッチ（解放）
- アタッチすることで、ボックスがアームと一緒に動く

### 3. 動作計画
- **関節空間**: `go_to_joint_state([j1, j2, j3, j4, j5, j6])`
  - 6つの関節角度を直接指定
  - 計算が速く、確実に到達可能
  
- **タスク空間**: `go_to_pose_goal(pose)`
  - XYZ位置と姿勢を指定
  - 逆運動学で関節角度を計算
  - より直感的

### 4. 姿勢の表現
- 位置: (x, y, z) メートル単位
- 姿勢: クォータニオン (w, x, y, z)
  - または オイラー角 (roll, pitch, yaw)
  - デフォルトは下向き: pitch=π

### 5. アプローチ戦略
```
上方でアプローチ → 降下 → グリップ → 上昇 → 移動
```
衝突を避けるため、上下動作を入れる

## RVizでの確認項目

1. **Planning Scene** - ボックスが表示されているか
2. **Trajectory** - 計画された軌道が表示される
3. **Robot Model** - アームの動きをリアルタイムで確認
4. **Motion Planning** - 手動でゴール姿勢を設定して計画も可能

## トラブルシューティング

### 計画が失敗する場合
- 目標位置がワークスペース外にある
- 逆運動学の解が存在しない
- 衝突が検出された

対処法：
```python
# 計画時間を増やす
self.move_group.set_planning_time(10.0)

# 試行回数を増やす
self.move_group.set_num_planning_attempts(20)

# 目標位置を調整する
```

### ボックスが表示されない場合
```python
# 待機時間を増やす
time.sleep(1.0)  # シーン更新待ち

# Planning Sceneをリフレッシュ
self.scene.remove_world_object(box_name)
self.add_box(box_name, position, size)
```

## 応用例

### 複数のボックスを運ぶ
```python
for i, (pos_a, pos_b) in enumerate(zip(points_a, points_b)):
    box_name = f"box_{i}"
    self.add_box(box_name, pos_a)
    # ピック＆プレース処理
```

### テーブルや障害物を追加
```python
# テーブル追加
self.add_box("table", (0.5, 0.0, -0.05), size=(0.6, 0.8, 0.1))

# 壁追加
self.add_box("wall", (0.0, 0.5, 0.3), size=(0.05, 1.0, 0.6))
```

### カスタム軌道
```python
# ウェイポイントを追加して滑らかな動作
waypoints = [pose1, pose2, pose3]
(plan, fraction) = self.move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0
)
self.move_group.execute(plan, wait=True)
```

## 参考資料

- [MoveIt Documentation](https://moveit.ros.org/)
- [MoveIt Commander Tutorial](https://ros-planning.github.io/moveit_tutorials/)
- [TM Robot Documentation](https://www.tm-robot.com/)
