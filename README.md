# rebot — Meta Quest 3 → ROS 2 テレオペレーション統合リポジトリ

Meta Quest 3 のコントローラを使い、ROS 2 (Jazzy) 経由で reBot Arm B601-DM
(Damiao motor 版) をテレオペレーションするための正式な統合リポジトリです。

```
Meta Quest 3
    ↓ CloudXR / OpenXR
NVIDIA IsaacTeleop (examples/teleop_ros2, release/1.3.x)
    ↓ ROS 2 topics / TF
quest_rebot_bridge  ← 本リポジトリでの新規実装
    ↓ ROS 2 (move_to_pose_ik)
Seeed reBotArmController_ROS2 (+ reBotArm_control_py)
    ↓
B601-DM
```

設計の詳細（採用外部リポジトリ、ROS 2 topic/message/TF 仕様、データフロー、
安全設計、Milestone計画）は [docs/architecture.md](docs/architecture.md)
を参照してください。

## 対象環境

- NVIDIA DGX Spark / GX10, Ubuntu 24.04, ARM64 (aarch64)
- Python 3.12
- ROS 2 Jazzy
- Meta Quest 3 (Developer Mode) + NVIDIA CloudXR
- 対象ロボット: reBot Arm B601-DM (Damiao motor 版) 1台
  （将来的に2台での二腕テレオペを想定した設計）

## リポジトリ構成

```
rebot/
├── README.md
├── docs/
│   └── architecture.md          # 設計ドキュメント（本体）
├── ros2_ws/
│   ├── rebot.repos              # vcstool: 外部リポジトリの取得定義
│   └── src/
│       ├── quest_rebot_bridge/  # 新規実装（唯一の自作ROS2パッケージ）
│       │   └── quest_rebot_bridge/controller_raw_publisher.py  # M1で実装済み
│       └── external/            # vcs import で取得（.gitignore対象、gitには含めない）
│           ├── IsaacTeleop/
│           └── reBotArmController_ROS2/
│               └── third_party/
│                   └── reBotArm_control_py/  # コントローラ側コードが要求する固定path
└── tools/
    └── xr_pose_smoke_test.py    # Quest→CloudXR→isaacteleopの単体疎通確認
                                  # （本番ROS2実装とは独立、ROS2化しない）
```

`reBotArm_control_py` は git submodule ではなく `rebot.repos` で
`reBotArmController_ROS2/third_party/reBotArm_control_py` に直接 vcs import
する（理由は [docs/architecture.md](docs/architecture.md) 参照）。

## ブランチ運用

- `main`: README・architecture・setup手順などの安定した内容
- `feature/quest-ros2-teleop`: 実装作業ブランチ

## 安全方針

**明示的な許可があるまで、以下は行いません:**

- B601-DM へモータ指令を送る (`move_to_pose_ik` 等の呼び出し)
- CAN / serial / motorbridge を open して実機を動かす
- `FollowJointTrajectory` 等、実機を動かす command の publish/call

実機接続前の安全条件（joint limits, workspace limits, max translation/rotation
step, max velocity, stale input timeout, tracking loss時停止, IK failure時停止,
clutch OFF時停止）の詳細設計は [docs/architecture.md](docs/architecture.md)
を参照してください。

## Milestone

| # | 内容 | 状態 |
|---|---|---|
| M0 | 環境構築（ROS 2 Jazzy install, 外部repo取得, colcon build） | 完了 |
| M1 | Quest controller data を ROS 2 topic として取得（reBot未起動） | 完了 |
| M2 | `quest_rebot_bridge` で target pose を計算・publish（実機未接続） | 完了 |
| M3 | reBot ROS 2 stack を非実機で起動し joint_states/TF/robot model 確認 | 完了 |
| M4 | Quest抜き・固定小 target pose で実機 command 経路を検証 | **完了** |
| M5 | Quest接続、低速・小スケール・clutch必須の片腕テレオペ初回 | **試行中（下記参照）** |

## 現在の状態（2026-09-14時点）と再開手順

**M4完了。M5（Quest実接続でのテレオペ）を試行したが、シミュレーション上で
腕が反応せず未達成。** 以下、次回再開時の要点。

### M4で完了したこと（実機、ユーザー許可の上で実施済み）

- B601-DM全7モータにCAN ID(0x01-0x07)/Master ID(0x11-0x17)をDM_Toolで設定
  （ツール本体は`assemble_rebot/motor-debugging-tool/`、vendorクローンにつき
  `.gitignore`対象）。
- 物理配線・24V電源投入・GX10側デバイス認識（`/dev/ttyACM0`）確認済み。
- `motorbridge` venv（`~/rebot/.venv-motorbridge/`、`--system-site-packages`で作成、
  `motorbridge`/`pin`/`numpy==1.26.4`固定でインストール — numpy 2.xは
  `tf_transformations`が依存する`transforms3d`と非互換なので注意）。
- ゼロ点キャリブレーション実施（組み立てガイドの折りたたみ姿勢を基準）。
  **重要な発見**: このゼロ点でjoint2・joint3は可動域の**上限**（実際の範囲は
  `[-180°, 0°]`）にちょうど乗っている。狭い意味で「動かせない」わけではないが、
  この境界ちょうどから小さく動かそうとするとIKの単発ソルバー（リトライなし）が
  収束しないケースがある（詳細はコミット`52c82c2`メッセージ参照）。
- `rebotarmcontroller`を実機接続で起動 → **接続した瞬間に全モータがenableされ、
  その場の姿勢を保持する500Hz制御ループが即座に開始する**（受動的な"繋ぐだけ"
  は存在しない、`hardware_manager.py`で確認済み）。
- `move_to_pose_ik`で実際に小さい実機動作を確認（joint1単独回転、joint2/3の
  複合動作）、`joint_states`で狙い通りの動きを検証済み。

### M5で分かったこと・残課題

- `bridge_node.py`をM2の「publish-onlyでハードウェア非接触」から拡張し、
  毎フレーム`move_to_pose_ik`を実際に呼ぶように変更済み（`arm_namespace`
  パラメータ、デフォルト`rebotarm`）。
- `fake_ik_driver.py`（新規）: 実機と同じURDF/IKソルバーを使うが
  CAN/serial一切なしの`move_to_pose_ik`互換ノード。`/rebotarm_sim`名前空間で
  起動し、実Quest 3の入力を実機に触れずにリハーサルできる
  （`sim_bringup.launch.py`）。
- 実際にQuest 3 → `bridge_node` → `fake_ik_driver`で試したが、**RViz上で
  腕が目に見えて反応しなかった**。分かっている要因:
  1. `quest_world → base_link`のキャリブレーションが未実施（identity
     placeholderのまま、`bridge.launch.py`の既知TODO）。回転の対応が
     ズレていると、手の動きが毎回同じ「詰まりやすい方向」に変換されてしまう
     可能性が高い（実際、境界姿勢から離れた場所での検証では6方向中5方向で
     IKは問題なく解けた）。
  2. `bridge_node`に、IK失敗時も内部の目標姿勢が握った手の動きに追従して
     ドリフトし続けるバグがあった（今回のセッションで発見・修正済み。
     成功時のみ内部参照を更新するように変更）。
- 次回はまず: (a) 境界姿勢ではない場所からclutchを握り直す、
  (b) `quest_world`の実キャリブレーションを行う（位置よりも回転が重要 —
  clutchは相対追従のみ使うため）、を優先する。

### 既存の実装・検証済み環境（このまま使える）

- `ros2_ws/`: `colcon build --symlink-install`済み
  （`rebotarmcontroller`, `rebotarm_msgs`, `rebotarm_bringup`, `quest_rebot_bridge`）
  — `rebotarmcontroller`/`quest_rebot_bridge`は`motorbridge`/`pin`依存のため
  `.venv-motorbridge`を有効化した状態でビルドする必要がある
  （`sys.executable`経由のcolcon実行、詳細は`docs/architecture.md`）。
- `~/rebot/.venv-teleop-ros2/`: `python3 -m venv --system-site-packages`で作成、
  `isaacteleop[cloudxr,retargeters]~=1.3.131 msgpack msgpack-numpy`をpip install済み
  （`controller_raw_publisher`用。詳細は[docs/architecture.md](docs/architecture.md)）
- Quest 3実機での動作確認済み: `controller_raw_publisher`（M1）、
  `bridge_node`（M2、squeeze clutch・相対追従・workspace clamp）
- `robot_state_publisher` + `joint_state_publisher_gui`での動作確認済み:
  `reBot_B601_DM_with_gripper.urdf`（M3、終端リンク名は`gripper_link`）
- 実機B601-DM: モータID設定・ゼロ点校正・`move_to_pose_ik`経由の実動作、
  すべて確認済み（M4）。セッション終了時は安全にdisable済み。

再現コマンドは全て[docs/architecture.md](docs/architecture.md)に記載。
