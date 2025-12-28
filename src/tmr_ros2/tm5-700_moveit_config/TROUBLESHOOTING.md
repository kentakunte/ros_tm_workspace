# RViz インタラクティブマーカー（青い球体）のトラブルシューティング

## 問題1：青い球体が追従しない

### 原因と解決策

#### 1. **Joint State Publisherが起動していない** ⭐最も一般的
**症状**: 青い球体をドラッグしても反応しない、またはロボットモデルが動かない

**原因**: `/joint_states`トピックが配信されていない

**解決策**: MoveItを使用する場合、`move_group`が自動的にjoint statesを配信するため、別途joint_state_publisherは**不要**です。

## 問題2：アームモデルが2つのポジション間で点滅する ⭐⭐⭐

### 症状
- ロボットアームが2つの位置間を高速で切り替わる
- 片方が表示されると片方が消える
- ループして点滅し続ける

### 原因
複数のノードが異なる`/joint_states`を同時に配信しているため、RVizが両方の状態を交互に表示しています。

**よくある原因**：
1. `joint_state_publisher_gui` と `move_group` が同時に動作
2. 複数の`joint_state_publisher`が起動
3. デモモードとコントローラーが競合

### 解決策

#### 方法1: joint_state_publisherを削除（推奨）

MoveItの`move_group`が既にjoint statesを配信するため、`joint_state_publisher`は不要です。

`demo.launch.py`を修正（修正済み）：

```python
# ❌ 削除: joint_state_publisher
# joint_state_publisher = Node(...)

return LaunchDescription([
    static_tf_node,
    robot_state_publisher,
    # joint_state_publisher,  # ← この行を削除
    run_move_group_node,
    rviz_node,
])
```

#### 方法2: 診断スクリプトで確認

```bash
# MoveItを起動した状態で実行
ros2 run tm5-700_moveit_config diagnose_blinking.py
```

出力例（問題あり）：
```
⚠ 警告: 2種類の異なる状態が配信されています
これが点滅の原因です！

状態1: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
  配信回数: 15回

状態2: [0.100, -0.200, 0.300, 0.000, 0.500, 0.000]
  配信回数: 14回
```

#### 方法3: ノードリストで確認

```bash
# 実行中のノードを確認
ros2 node list

# joint_state関連のノードを探す
ros2 node list | grep -i joint

# 以下が表示される場合は問題
# /joint_state_publisher
# /joint_state_publisher_gui
```

**対処**：余分なノードを停止するか、launchファイルから削除

#### 方法4: トピックの配信者を確認

```bash
# /joint_statesの配信者を確認
ros2 topic info /joint_states

# 出力例（正常）:
# Publishers: 1
#   * /move_group (sensor_msgs/msg/JointState)

# 出力例（問題あり）:
# Publishers: 2  ← 2つ以上あると点滅の原因
#   * /move_group
#   * /joint_state_publisher_gui
```

### 修正後の確認

```bash
# 再ビルド
cd ~/ros2_ws
colcon build --packages-select tm5-700_moveit_config

# 起動
source install/setup.bash
ros2 launch tm5-700_moveit_config demo.launch.py
```

**期待される動作**：
- ✅ アームモデルが単一の位置で安定表示
- ✅ インタラクティブマーカーで滑らかに動く
- ✅ 点滅なし

---

## 問題3：Joint State Publisherが起動していない
**症状**: 青い球体をドラッグしても反応しない、またはロボットモデルが動かない

**原因**: `/joint_states`トピックが配信されていない

**解決策**: `demo.launch.py`を修正済み。以下を確認：
```bash
# 再ビルド
cd ~/ros2_ws
colcon build --packages-select tm5-700_moveit_config

# 起動
source install/setup.bash
ros2 launch tm5-700_moveit_config demo.launch.py
```

別ターミナルで確認：
```bash
# joint_statesトピックが配信されているか確認
ros2 topic echo /joint_states

# 期待される出力：
# name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
# position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

#### 2. **TF（座標変換）の問題**
**症状**: "Fixed Frame [world] does not exist" エラー

**確認方法**:
```bash
# TFツリーを確認
ros2 run tf2_tools view_frames
# frames.pdfが生成される

# 特定のTFを確認
ros2 run tf2_ros tf2_echo world base
```

**解決策**: Static TF publisherが起動されているか確認（修正済み）

#### 3. **RVizの設定問題**
**症状**: 青い球体が見えない、またはエラー表示

**解決策**:
1. RVizの左下「Fixed Frame」を確認
   - `world` または `base` に設定

2. MotionPlanningプラグインの設定確認
   - "Planning Request" → "Query Start State" をチェック
   - "Query Goal State" をチェック
   
3. Interactive Markerの表示確認
   - "Planned Path" → "Show Robot Visual" をON
   - "Scene Robot" → "Show Robot Visual" をON

#### 4. **move_groupノードの問題**
**確認方法**:
```bash
# move_groupノードが起動しているか
ros2 node list | grep move_group

# move_groupのログを確認
ros2 node info /move_group
```

**ログに以下があるか確認**:
- ✓ "Loading robot model 'tm5-700'..."
- ✓ "Ready to take commands"

#### 5. **Planning Sceneの更新問題**
**症状**: ロボットは動くが、青い球体がおかしい

**解決策**:
```bash
# Planning Sceneトピックを確認
ros2 topic hz /planning_scene
ros2 topic hz /move_group/monitored_planning_scene
```

## 診断スクリプトの実行

```bash
# MoveItを起動した状態で別ターミナルで実行
ros2 run tm5-700_moveit_config moveit_diagnostics.py
```

このスクリプトが以下をチェックします：
- ✓ /joint_states トピック
- ✓ /planning_scene トピック
- ✓ /tf, /tf_static トピック
- ✓ Interactive Markerトピック

## 正常動作時の確認項目

### RViz上での確認
1. **青い球体が表示される場所**
   - Start State: ロボットの現在位置
   - Goal State: 目標位置（オレンジ色のロボットと一緒）

2. **操作方法**
   - 球体をドラッグ: 並進移動
   - 輪をドラッグ: 回転
   - 右クリック: メニュー表示

3. **動作確認**
   - 球体を動かす → オレンジ色のロボットが追従する
   - "Plan" ボタン → 軌道が表示される
   - "Execute" ボタン → ロボットが動く

### トピック確認
```bash
# 全トピックをチェック
ros2 topic list

# 必須トピック
/joint_states                              # ✓ 関節状態
/move_group/display_planned_path           # ✓ 計画パス
/planning_scene                            # ✓ シーン情報
/tf                                        # ✓ 動的TF
/tf_static                                 # ✓ 静的TF

# Interactive Marker関連
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
```

## よくある間違い

### ❌ 間違い1: use_sim_time=Trueで起動
実機やシミュレーターなしで`use_sim_time: True`を設定すると、タイミングがずれる

**修正**: demo.launch.pyから`use_sim_time`を削除（修正済み）

### ❌ 間違い2: joint_state_publisherなしで起動
joint_statesが配信されないと、インタラクティブマーカーが機能しない

**修正**: joint_state_publisher_guiを追加（修正済み）

### ❌ 間違い3: Static TFが未設定
world → base のTFがないと、RVizが座標系を確立できない

**修正**: static_transform_publisherを追加（修正済み）

## 詳細デバッグ

### ログレベルを上げる
```bash
ros2 launch tm5-700_moveit_config demo.launch.py --log-level DEBUG
```

### RVizの再起動
```bash
# MoveItは動いたまま、RVizだけ再起動
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix tm5-700_moveit_config)/share/tm5-700_moveit_config/config/moveit.rviz
```

### トピックの可視化
```bash
# rqt_graphで全体像を確認
rqt_graph

# トピックの周波数をモニター
ros2 topic hz /joint_states
ros2 topic hz /tf
```

## まとめ

修正後の起動で以下が実行されます：
1. ✅ Static TF Publisher (world → base)
2. ✅ Robot State Publisher (URDFからTF配信)
3. ✅ Joint State Publisher GUI (関節状態を配信)
4. ✅ move_group (MoveItの中核)
5. ✅ RViz2 (可視化)

これで青い球体が正しく追従するはずです！
