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
| M4 | Quest抜き・固定小 target pose で実機 command 経路を検証 | 未着手 |
| M5 | Quest接続、低速・小スケール・clutch必須の片腕テレオペ初回 | 未着手 |
