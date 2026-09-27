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
- 対象ロボット: reBot Arm B601-DM (Damiao motor 版) 2台（左右）

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
| M5 | Quest接続、低速・小スケール・clutch必須の片腕テレオペ初回 | **完了（下記参照）** |
| M6 | 実機B601-DM 2台（左右）でのQuest二腕テレオペ | **完了（2026-09-27）** |

## 現在の状態（2026-09-27時点）

**M6完了。** 実機B601-DM 2台（左右）を、実Quest 3の左右コントローラーで
同時にテレオペできることを2026-09-27に確認した。起動手順・セットアップは
下記「[実機二腕テレオペ（M6）](#実機二腕テレオペm62026-09-27)」を参照。

M5（2026-09-26）までは、実Quest 3 + `dual_arm_rehearsal.launch.py`
（シミュレーション）でのテレオペを確認していた。操作方法は、squeezeクラッチに
よる相対位置追従、エンドエフェクタ姿勢は常に鉛直下向き固定、ロールのみ
右スティックで操作。途中で直した問題（`ee_frame`不一致、`quest_world`
キャリブレーション未実施、IK失敗時に目標姿勢がドリフトするバグ、姿勢固定化と
home姿勢の不整合、大学Wi-Fiの輻輳など）の詳細は、以下の各日付セクションを参照。

## 実機二腕テレオペ（M6、2026-09-27）

`launch/dual_arm_real.launch.py` を使う。構成は `dual_arm_rehearsal.launch.py`
と同じで、左右それぞれの `fake_ik_driver` を実機の `reBotArmController` に
置き換えたもの。

### 一度だけ必要なセットアップ

1. **モータID（DMTool）**: 左右とも同じIDでよい（CAN ID `0x01`〜`0x07` /
   Master ID `0x11`〜`0x17`、joint1〜6・gripperの順）。アダプタが左右で
   別なのでCANバスも別になり、IDがかぶっても衝突しない。新品のモータは全部
   `0x01`/`0x00` なので、**1個ずつつないで**設定する。DMToolは
   `assemble_rebot/motor-debugging-tool/Linux/arm64/DMTool-v2.1.8.1-arm64/start.sh`
   （vendorクローン、`.gitignore`対象）。
2. **ゼロ点（DMTool）**: アームを組み立てガイドの折りたたみ姿勢にし、
   Debugタブの **Save Zero（保存零点）** を7個それぞれで押す。グリッパーは
   閉じた状態で行う。確認するには、折りたたみ姿勢でモーター位置を読み、
   全部0°付近であればよい（2026-09-27の実測は左右とも±5°以内）。
3. **USBポートの固定名（udev）**: 2つのUSB-CANアダプタはシリアル番号が同じ
   （`00000000050C`）なので、**挿しているUSBポートで区別する**。
   ```bash
   sudo cp tools/udev/99-rebot-arms.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ls -l /dev/rebot_*   # rebot_left / rebot_right ができていればOK
   ```
   左＝`NVDA8000:01`、右＝`NVDA8000:02` のポート。**別のポートに挿すと名前が
   つかない**。確認は `udevadm info -q property -n /dev/ttyACM0 | grep ID_PATH=`。

### 起動手順

事前に、アームを左右とも折りたたみ姿勢にし、24V電源をONにする。DMToolは
閉じておく（ポートを使っていると制御プログラムがつなげない）。QuestとPCは同じ
ネットワークにつなぐ（テザリング推奨、下記M5の手順を参照）。

```bash
# ターミナル1: 実機2台 + bridge_node(左右) + RViz
source ~/rebot/.venv-motorbridge/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/rebot/ros2_ws/install/setup.bash
ros2 launch quest_rebot_bridge dual_arm_real.launch.py translation_scale:=0.5 \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_dual_arm_real.log
```

```bash
# ターミナル2: controller_raw_publisher（isaacteleop venv、Questと接続）
source /opt/ros/jazzy/setup.bash
source ~/rebot/.venv-teleop-ros2/bin/activate
cd ~/rebot/ros2_ws/src/quest_rebot_bridge
python3 -m quest_rebot_bridge.controller_raw_publisher \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_controller_raw.log
```

Quest側では、ヘッドセット内ブラウザでCloudXRクライアントを開き、Server IP
（`ip -4 addr show wlP9s9`）とPort `48322` を手入力して接続する。

**起動時の挙動（注意）**: 起動すると両腕のモータが即座にONになり、その場の
姿勢を保持する。約1秒後にグリッパーが開き、続いて**両腕が自動で
折りたたみ姿勢からホーム位置へ移動する**（`home_on_startup`、約2秒）。
起動前にアーム周辺に人や物がないことを確認すること。自動移動を止めたい
場合は `home_on_startup:=false` を付ける。折りたたみ姿勢のまま握っても
IKが解けないので、その場合は先にA/Xでホームへ戻す。

**操作**:
- グリップを握っている間: 手の動きに追従する（`translation_scale`倍）
- 離す: その場で保持する
- トリガー: グリッパーを開閉する
- 右スティックの左右: 手首ロールを回す
- A（右）/X（左）: ホームへ戻る

**終了**: ターミナル1で `Ctrl-C`。各アームが折りたたみ姿勢へ戻ってから
モータがOFFになる。

主なlaunch引数:
- `translation_scale`（既定1.0、上の例では0.5）
- `home_on_startup`（既定true）
- `left_channel`/`right_channel`（既定 `/dev/rebot_left`/`/dev/rebot_right`）
- `{left,right}_calib_q{x,y,z,w}`
- `fake_hardware`（下記）
- `rviz`

### キャリブレーション（左右別）

`quest_world → {side}/base_link` の回転は、**左右それぞれ実機で測った値**を
launchの既定値にしている（2026-09-27測定、左右で約27°違う）。この日の値は
2026-09-16の値（`dual_arm_rehearsal`が今も使っている共通の値）から約65°
ずれていた。Questのガーディアン再設定や向きのリセットをしたら、左右とも
測り直す。

測り直しは、アームのlaunchを止め（A/Xを押すとホームに戻る動作が始まるため）、
`controller_raw_publisher`は起動したままで行う:

```bash
ros2 run quest_rebot_bridge quest_world_calibrate --ros-args -p arm_side:=right
# Aを押しっぱなしで、右アームが前に伸びる方向へ水平に15cm以上押し出して離す
ros2 run quest_rebot_bridge quest_world_calibrate --ros-args -p arm_side:=left
# Xで同様に、左アームの前方向へ
```

表示された値は、`right_calib_qx:=... right_calib_qy:=...`（左も同様）の形で
launch引数に渡す。常用する値なら、`dual_arm_real.launch.py`の既定値を
書き換える。

### 実機を使わないリハーサル

```bash
ros2 launch quest_rebot_bridge dual_arm_real.launch.py fake_hardware:=true
```

実機の制御プログラムと**同じモデル**（`ReBot_Arm_DM.urdf` / `end_link`、
折りたたみ姿勢から開始）の `fake_ik_driver` に置き換えて、同じ構成を起動する。
2026-09-27に、折りたたみ姿勢からのホーム移動と左右±XYZの追従を、
`tools/check_dual_arm_tracking.py` をこのモード向けにした版で確認済み。

### M6で見つけて直した問題

- **`end_link`と`gripper_link`は向きが180°違う**: 実機の制御プログラムは
  `ReBot_Arm_DM.urdf`の`end_link`に対してIKを解く。一方、`bridge_node`は
  `gripper_link`の姿勢を送っている。2つは位置は同じだが、ローカルX軸まわりに
  180°回っている（FKで確認）。補正なしだと、実機のIKはほぼ失敗し、解けても
  1ステップでjoint6が約180°跳ねる（シミュレーションで確認）。
  `bridge_node`に`ik_orientation_offset_xyzw`（既定は恒等回転、
  `dual_arm_real`では`[1,0,0,0]`）を追加し、IKサービスへ送る姿勢にだけ掛ける。
- **SDKはポート名が`/dev/tty`で始まるときだけUSBシリアルとして扱う**:
  `reBotArm_control_py/actuator/rebotarm.py`。`/dev/rebot_left`のような名前を
  そのまま渡すとSocketCANのインターフェース名と解釈され、
  `if_nametoindex failed ... No such device` で起動直後に落ちる。
  `dual_arm_real`はlaunch時に`realpath`で`/dev/ttyACM*`へ変換してから渡す。
- 2台を同時に動かすため、`dual_arm_real`では左右の`reBotArmController`の
  ノード名を`reBotArmController_left`/`_right`、名前空間を`rebotarm_left`/
  `_right`に分けている。

## 過去の記録（M4〜M5）

以下は2026-09-15時点の記録（当時の未解決事象の詳細、その後すべて解決済み）。

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

### 2026-09-15セッション: MoveItリハーサル環境・Aボタンリセット・キャリブレーションツール

- `fake_ik_driver`（無ビジュアル・IKソルバーのみ）の代わりに、
  `reBotArmController_ROS2`同梱の`rebotarm_moveit_config`（MoveIt 2、
  `model:=dm`がデフォルトで実機と同じ`reBot_B601_DM_with_gripper.urdf`＋
  `gripper_link`終端を使用、既存パッケージで新規実装不要）をRViz可視化つき
  リハーサル環境として採用。`ros-jazzy-moveit`一式をapt導入し、
  `rebotarm_moveit_config`/`rebotarm_moveit_demos`をビルド済み。
- 新規ノード`moveit_ik_bridge`（`quest_rebot_bridge`）: `fake_ik_driver`と同じ
  IKソルバーを使うが、結果を`rebotarm_moveit_config`のros2_control
  mock hardware（`/rebotarm_controller/joint_trajectory`）に送ることで
  RViz上に反映させる。`/rebotarm/move_to_pose_ik`を公開し、`bridge_node`の
  デフォルト`arm_namespace`（`rebotarm`）とそのまま噛み合う。
  まとめ起動用に`launch/moveit_rehearsal.launch.py`を追加。
  単体サービス呼び出しでのIK→`/joint_states`反映は確認済み。
- `bridge_node`にAボタン（右手コントローラ、左手なら X）での
  ホームリセット機能を追加。押すとclutchを解除し、
  `home_position`/`home_orientation`へ既存の`safety_limits`のステップ制限
  （急な飛びなし）で戻る。squeeze再握りで即キャンセル。
  合成`controller_data`でのエンドツーエンドテスト済み。
- 新規ツール`quest_world_calibrate`: `quest_world → base_link`の**回転のみ**を
  較正する対話式ノード。数式で確認済み: `Clutch.compute_target()`は差分
  （`c - r`）しか使わないため、剛体変換の並進成分は差分計算で必ず打ち消される
  → 並進オフセットはteleopの向き・スケールに一切影響しない
  （`calib_x/y/z`は0のままでよい）。手順はロボット正面(+X)方向へ
  コントローラを構えてAボタン2秒長押し→算出した`calib_qx/qy/qz/qw`を
  `bridge.launch.py`に渡す。
- **新たな未解決事象**: 上記環境で実際にQuest 3を接続し、bridge_node経由で
  squeezeを握って動かしてみたが、**RViz上で全く動かなかった**（M5当初の
  「見た目の反応なし」と同じ現象で、方向が違うのではなく無反応）。
  原因未特定。次回確認すべき候補:
  - `_transform_to_base()`（quest_world→base_link）が失敗して即
    force-release されていないか（`bridge_node`のログで
    "transform unavailable" / "clutch force-released" を確認）
  - `move_to_pose_ik`呼び出しがIK非収束で毎回`success=False`になっていないか
    （`_on_move_ik_response`の warn ログを確認）
  - `squeeze_engage_threshold=0.5`に対し実機の`squeeze_value`が閾値に届いて
    いるか（Questのグリップボタンの効き方次第では届かない可能性）
  - **ハマりどころ**: `moveit_ik_bridge`や`bridge_node`を複数プロセス
    多重起動すると同じサービス名を取り合って誰にも正しく届かなくなる。
    再現時は`ps aux | grep -E "moveit_ik_bridge|bridge_node"`で
    重複がないか必ず確認してから切り分けること。

### 2026-09-16セッション: 「全く反応しない」の原因特定・修正

診断ログ（後述）を使って上記の無反応事象のログを分析した結果、原因を特定した:

- **`bridge_node`の`ee_frame`デフォルトが`gripper_link`だったが、
  `move_to_pose_ik`は（実機の`rebotarm_hardware.yaml`（`model=dm`、
  `end_effector_frame: end_link`）、`fake_ik_driver`、`moveit_ik_bridge`の
  **3つとも一致して**`end_link`（グリッパー無しの手前のリンク）を対象に
  IKを解いている。`gripper_link`は`end_link`から並進[0.26, 0, 0.192]m +
  X軸180°回転オフセットしたフレーム（M3調査で確認済み）のため、
  `bridge_node`が計算する目標姿勢は常にこの分だけズレた「ほぼ解けない」
  座標をIKソルバーに要求し続けていた。
- ログでの裏付け: `moveit_ik_bridge`の`[heartbeat]`で`calls_total=5402`
  中`success_total=1776`（約67%が`IK did not converge`で拒否）、
  `bridge_node`側では`home reset requested`が13回発生した一方
  `home reset complete`は**0回**（home_orientationがidentityのため、
  ズレた向きでのIK要求がほぼ常に失敗していたと一致）。
- **修正**: `bridge_node`の`ee_frame`デフォルトを`gripper_link`から
  `end_link`に変更（`bridge_node.py`）。これにより`move_to_pose_ik`が
  実際に解く座標系と`bridge_node`が追跡・要求する座標系が一致する。
  **未検証**: 次回Quest接続での再検証が必要。

### 2026-09-16セッション: 2腕(左右コントローラー→2アーム)対応

シミュレーション上で2本のreBotアームを左右コントローラーそれぞれで
同時に操作できるようにした。新規launch: `launch/dual_arm_rehearsal.launch.py`

- `moveit_rehearsal.launch.py`(MoveIt本体のmove_group/controller_manager)は
  2台分の名前空間分離が外部リポジトリの改修を要するため不採用。代わりに
  `fake_ik_driver` + `robot_state_publisher`のペア(M5当初の`sim_bringup.launch.py`
  と同じ軽量構成)を2組動かす方式にした。`fake_ik_driver`にもグリッパー
  (`gripper/open`/`gripper/close`)を追加済み。
- TFは`{side}/base_link`(`left/base_link`/`right/base_link`)を
  `robot_state_publisher`の`frame_prefix`で分離(区切り文字が`/`である
  理由は下記のレンダリング不具合の項を参照)。**`quest_world`は左右で
  共有せず、`{side}/quest_world`として各`{side}/base_link`の直接の子にした**
  (1つの`quest_world`を共有すると親子関係上どちらか一方の腕側だけが親になり、
  もう片方向けには較正回転の**逆**を計算する必要が生じ、ee_frame/RUB-FLUの
  ようなミスを誘発しやすいため回避)。物理的に左右対称配置であれば、
  同じ較正値をそのまま両方の`{side}/quest_world`に使い回せる(並進は
  無関係、回転だけが問題になるのは今までと同じ)。
- 見た目の位置合わせ用に`world`という第三の frame を追加
  (`world -> left/base_link` / `world -> right/base_link`、Y方向に
  ±0.4mオフセット)。**実測値ではなく表示用の適当な間隔**で、
  bridge_nodeの計算には一切関与しない。
- RViz表示は自作の`.rviz`設定(`rviz/dual_arm.rviz`)を追加、2つの
  `RobotModel`表示(それぞれ`/left/robot_description`・
  `/right/robot_description`トピック、TF Prefix`left`/`right`)を
  1つのウィンドウに同時表示。
- `bridge_node`を`arm_side:=left`/`right`で2つ起動(このlaunch内に
  直接組み込み済み、別途`bridge.launch.py`を起動する必要はない)。
- **実機テスト時の落とし穴を1つ発見・修正**: `.venv-motorbridge`を
  activateしただけで`colcon build`すると、colcon自体がシステムPythonで
  動くため生成される実行ファイルのshebangがシステムPythonのままになり
  `ModuleNotFoundError: No module named 'motorbridge'`で実行時に落ちる。
  `python3 -m colcon build ...`(venv activate後、python3コマンド経由で
  colconを呼ぶ)が必須（詳細は上記「実行コマンド」セクション参照）。
- dry-run検証済み(実機なし、Quest未接続の状態でlaunchのみ起動): 両
  `fake_ik_driver`起動、両`bridge_node`起動時に1/3開度でのgripper open成功
  (`target=0.0238m`)、TF `world -> right/gripper_link`が期待通りの座標
  (0.260, -0.400, 0.192、回転X軸180°)になることを確認。
- **見つけて直したレンダリング不具合**: 初回実装時、RVizのDisplaysパネルで
  `LeftArm`/`RightArm`(RobotModel表示)が赤エラーになり、3Dビュー上に
  メッシュが出ずTFの軸ラベルだけが見える状態だった(自分でスクリーンショットを
  撮って`Status: Error`を展開して確認: `No transform from [left_/base_link]`
  のように**スラッシュが余分に入った**フレーム名を探していた)。原因は
  `robot_state_publisher`の`frame_prefix`パラメータ(単純な文字列結合、
  `"left_"+"base_link"="left_base_link"`)と、RVizのRobotModel表示の
  「TF Prefix」プロパティ(`tf_prefix + "/" + link_name`で**自動的にスラッシュを
  挿入**)が結合方法が違うこと。フレーム名の区切りを`_`ではなく`/`
  (`left/base_link`のような形)に統一し、`frame_prefix`を`"left/"`、
  RVizの`TF Prefix`を`"left"`(スラッシュなし)にすることで解決。
  修正後、スクリーンショットで2本のアームのメッシュが正しく描画されている
  ことを確認済み。
- **Quest実接続での実地テストは未実施**。

### 2026-09-16セッション: グリッパー制御(二値)を実装

人差し指トリガーでグリッパーを開閉できるようにした（今まで`sample.trigger`
は decode されるだけで未使用だった）。

- `bridge_node.py`: トリガー値に対し閾値ヒステリシス
  (`trigger_close_threshold=0.5` / `trigger_open_threshold=0.4`、squeeze
  clutchと同じ方式)で`gripper/open`/`gripper/close`serviceを**閾値を
  跨いだ瞬間に1回だけ**呼ぶ。両serviceは実機側(`hardware_manager.py`の
  `set_gripper_position`)で最大3秒ブロッキングする設計のため、
  `move_to_pose_ik`のように毎フレーム呼ぶことはできない
  （呼んだ瞬間に`_gripper_closed`をoptimisticに更新し、応答を待たずに
  次のフレームでの重複呼び出しを防止）。
- 「開く」側は全開ではなく**全開の1/3**の位置(`gripper_open_position`
  パラメータ、`rebotarm_hardware.yaml`のdmモデル`open: -5.0rad`から
  `-5.0/3 ≈ -1.667rad`を計算)をデフォルトにした
  （実機のグリッパーは全開だと大きく開きすぎるため、ユーザー指定）。
  「閉じる」側は実機側のデフォルト(`close: 0.0rad`)をそのまま使う。
  ノード起動1秒後に一度「開く」を自動実行し、初期状態を確定させる。
- **RVizリハーサルでも見えるように`moveit_ik_bridge.py`にも同じ
  `gripper/open`/`gripper/close`serviceを追加**。`rebotarm_moveit_config`
  のURDFが持つ別コントローラ`gripper_controller`
  (`gripper_joint1`/`gripper_joint2`、範囲`[0, 0.0715]m`)に
  JointTrajectoryを送る。**注意**: 実機の`position`(ラジアン、
  1軸のモーター駆動)とこのURDFのプリズマティック関節(メートル、
  2軸)は単位もスケールも違うため、`request.position`はここでは無視し、
  同じ「1/3開く」という考え方だけをこのシミュレーション独自の単位で
  再現している（見た目の確認用、実機の正確な開度予測ではない）。
- `[heartbeat]`ログに`gripper_closed=...`を追加済み。

### 2026-09-16セッション: 「方向が全然違う」の原因特定・`quest_world_calibrate`再設計

Aボタン修正後の再検証で、clutch握って真上に手を上げるテストをして
`[motion]`ログを分析した結果:

```
delta_quest = (0.018, 0.788, 0.031)   # Quest生データ: ほぼ上(+Y)方向のみ
delta_base  = (0.042, 0.670, -0.413)  # 較正後: 横(+Y)と下(-Z)に分散、上(Z)に乗っていない
```

較正の回転行列を`delta_quest`に実際に適用した結果は`delta_base`と完全一致
（`R @ delta_quest ≈ delta_base`）——つまり`bridge_node`の変換コード自体は
正しく動いている。バグは**較正ツール(`quest_world_calibrate`)が測定する
回転の値そのもの**にあった。

**根本原因**: 旧`quest_world_calibrate`は「コントローラを正面に向けて2秒
保持」した**その瞬間の生の向き(orientation)そのもの**をidentity(=
base_linkの軸)とみなす設計だった。しかしQuestの座標系はOpenXR RUB
（X=右, Y=上, Z=後ろ）、`base_link`はROSのFLU（X=前, Y=左, Z=上）で、
**「上」が何番目の軸か自体が違う**。旧ツールは前方向(yaw)のズレは補正
できるが、RUB→FLUの軸の入れ替えは一切行っていなかった。ユーザーが
手順通り水平に正確に構えても、この設計では原理的に直らない
（`bridge_node.py`のdocstringが「軸変換もこの変換に含まれている」と
誤って書いていたのが実態と食い違っていた）。

**修正**: `quest_world_calibrate.py`を再設計。orientationの保持ではなく、
**「ロボットに向かって水平に押し出す」という移動(position)**から前方向を
測定する方式に変更。Questの上方向(+Y)は仕様上常に重力方向で測定不要のため
固定でZ軸に割り当て、測った前方向と外積で求めた左方向を合わせて回転行列を
構成する（controller-local軸の規約に一切依存しない、より頑健な方法）。
新しい起動時の指示は「Aボタンを押しっぱなしにしながらロボット方向へ
15cm以上水平に押し出して離す」に変更。

**`bridge.launch.py`のデフォルトはいったんidentity(placeholder)に戻した後、
新方式で再較正し直した値(30.6cmの水平プッシュ、`calib_qx=0.558549,
calib_qy=-0.433616, calib_qz=-0.433616, calib_qw=0.558549`)を反映済み**。
検証済み: この回転をQuestの上方向(+Y)に適用するとbase_linkのZ軸
(上)に横方向への漏れゼロで一致することを確認した。次回はこの値のまま
テレオペ再検証(clutch握って真上に持ち上げるテスト→`[motion]`ログで
delta_baseのZ成分が支配的になっているか確認)から始めればよい。

### 2026-09-16セッション: 診断ログの追加

上記「無反応」の原因を次回セッションでログだけから切り分けられるように、
`quest_rebot_bridge`に以下を追加（再ビルド不要、`--symlink-install`済みのため
ソース変更は即反映される）:

- `bridge_node`: 1秒ごとに`[heartbeat]`ログ（`controller_data_age`,
  `is_active`, `squeeze`, `clutch_engaged`, `homing`, `last_ik_call_success`）
  を出力。以前は状態が変化した時だけログしていたため、「完全無反応」の場合に
  ログが一切出ず`controller_data`未受信なのか受信しているが握り判定に届かないのか
  区別できなかった。
- `moveit_ik_bridge`: それまで**成功も失敗も一切ログしていなかった**。
  呼び出し回数・成功回数・直近の成否を1秒ごとに出力し、IK非収束時はwarnログを
  追加。
- `node_diagnostics.py`（新規、両nodeで共有): 起動時のROSグラフを見て
  同名nodeが複数存在したらERRORログを出す（上記「多重起動」を`ps aux`の
  手動確認なしで検知）。

### 次回起動手順（ターミナル別、M5リハーサル再現用）

**前提**: どのターミナルでも最初に`source /opt/ros/jazzy/setup.bash`が必要。
ターミナル1・2は追加で`source ~/rebot/ros2_ws/install/setup.bash`も必要
（`isaacteleop`のvenvは不要 — venvを有効化したままのターミナルで実行すると
`Package 'quest_rebot_bridge' not found`になるので、別タブ/別ターミナルを使うこと）。

```bash
mkdir -p ~/rebot/logs   # 初回のみ
```

**ターミナル1: MoveItデモ(RViz) + moveit_ik_bridge**
```bash
source /opt/ros/jazzy/setup.bash
source ~/rebot/ros2_ws/install/setup.bash
ros2 launch quest_rebot_bridge moveit_rehearsal.launch.py \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_moveit.log
```
確認: RViz起動、`moveit_ik_bridge up: ...`ログ、以降1秒ごとに`[heartbeat] calls_total=...`。

**ターミナル2: quest_worldキャリブレーションTF + bridge_node**

2026-09-16に実測した`calib_qx/qy/qz/qw`が`bridge.launch.py`の**デフォルト値**
として既に埋め込み済みなので、通常は引数なしでそのまま起動できる。
```bash
source /opt/ros/jazzy/setup.bash
source ~/rebot/ros2_ws/install/setup.bash
ros2 launch quest_rebot_bridge bridge.launch.py \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_bridge.log
```
確認: `quest_rebot_bridge_node up: ...`ログ、以降1秒ごとに`[heartbeat] controller_data_age=...`。

再校正が必要になった場合（ロボット本体を物理的に動かした、またはQuestの
Guardian/room setupを再設定した場合のみ — 詳細は下記「quest_world_calibrate」）
は、出力された値を`bridge.launch.py`の起動引数で上書きするか、デフォルト値
自体を書き換える:
```bash
ros2 launch quest_rebot_bridge bridge.launch.py \
  calib_qx:=<値> calib_qy:=<値> calib_qz:=<値> calib_qw:=<値>
```

**ターミナル3: controller_raw_publisher（isaacteleop venv、Questと接続）**
```bash
source /opt/ros/jazzy/setup.bash
source ~/rebot/.venv-teleop-ros2/bin/activate
cd ~/rebot/ros2_ws/src/quest_rebot_bridge
python3 -m quest_rebot_bridge.controller_raw_publisher \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_controller_raw.log
```
`CloudXR runtime launching...`が出たらQuest 3を装着し、ヘッドセット内ブラウザで
CloudXRクライアントに接続（`tools/xr_pose_smoke_test.py`と同じ手順）。
確認: `TeleopSession started successfully`。

**(再キャリブレーションが必要な場合のみ) quest_world_calibrate**

2026-09-16に較正方式を再設計・再実施済み（新方式の値が`bridge.launch.py`
のデフォルトに反映されている）。ロボット本体を物理的に動かした、または
QuestのGuardian/room setupを再設定した場合のみ再実施すればよい。
ターミナル3が起動しQuest接続済みの状態で、別ターミナルで:
```bash
source /opt/ros/jazzy/setup.bash
source ~/rebot/ros2_ws/install/setup.bash
ros2 run quest_rebot_bridge quest_world_calibrate
```
**Aボタン(右手)を押しっぱなしにしながら、ロボットに向かって水平に
15cm以上コントローラを押し出して、離す**（向きを保持する旧方式ではない）。
出力された`calib_qx/qy/qz/qw`を`bridge.launch.py`の`DeclareLaunchArgument`
デフォルト値に書き換える（またはターミナル2のコマンドに起動引数として渡す）。

**操作**: squeeze握り = clutch engage（ターミナル2ログに`clutch engaged`）、
握ったまま動かす = 相対追従、離す = その場保持、A/Xボタン = home復帰。

**トラブルシュート**: 反応がなければ`~/rebot/logs/`の3ファイルを
`grep -E "heartbeat|WARN|ERROR"`で確認。`controller_data_age`が伸び続ける→
ターミナル3側の問題、`clutch_engaged=False`のまま→squeeze値かTF変換の問題、
`last_ik_call_success=False`が続く→IK非収束、`duplicate node detected`→
多重起動（`ps aux | grep -E "moveit_ik_bridge|bridge_node"`で確認して片方kill）。

### 既存の実装・検証済み環境（このまま使える）

- `ros2_ws/`: `colcon build --symlink-install`済み
  （`rebotarmcontroller`, `rebotarm_msgs`, `rebotarm_bringup`, `quest_rebot_bridge`,
  `rebotarm_moveit_config`, `rebotarm_moveit_demos`）
  — `rebotarmcontroller`/`quest_rebot_bridge`は`motorbridge`/`pin`依存のため
  `.venv-motorbridge`を有効化した状態でビルドする必要がある。
  **注意(2026-09-16判明)**: `source .venv-motorbridge/bin/activate`しただけで
  普通に`colcon build`すると、colcon自体がシステムPythonで動いているため
  生成される実行ファイルのshebangがシステムPython(`/usr/bin/python3`)のままになり、
  実行時に`ModuleNotFoundError: No module named 'motorbridge'`で落ちる。
  必ず**`python3 -m colcon build ...`**(venv有効化後、`python3`コマンド経由で
  colconを起動する)を使うこと。shebangが`.venv-motorbridge/bin/python3`に
  なっているか`head -1 install/quest_rebot_bridge/lib/quest_rebot_bridge/<実行ファイル名>`
  で確認できる。
  `ros-jazzy-moveit`一式（`ros-jazzy-moveit`, `ros-jazzy-controller-manager`,
  `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-joint-trajectory-controller`,
  `ros-jazzy-ros2-control(lers)`, `ros-jazzy-rviz2`ほか）をaptで導入済み。
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

### 二腕シミュレーション：コントローラー姿勢への追従（2026-09-16更新）

二腕launchは位置をクラッチ相対操作、姿勢を絶対追従に変更した。
握り直しても姿勢の対応は変わらない。姿勢にはコントローラーの指す方向を表す
OpenXR `aim_orientation`、位置には従来の`grip_position`を使う。
前方向はコントローラーの−Zとグリッパーの+X、上方向はコントローラーの+Yと
グリッパーの−Zを対応させる（固定補正xyzw = `[0.5, 0.5, -0.5, 0.5]`）。
この対応は`controller_to_tool_xyzw`パラメータで変更できる。
`orientation_mode=absolute`では回転倍率を使わず、速度・ステップ制限は維持する。
単腕launchの既定は相対姿勢操作のまま、回転軸をbase座標系で一致させた。

IKとRVizは同じ`reBot_B601_DM_with_gripper.urdf`の`gripper_link`を使う。
左右のTF名は`left/gripper_link`、`right/gripper_link`。
手先TFがない間は操作を開始せず、固定ホーム姿勢で代用しない。
シミュレーションは到達確認済みのホーム位置`[0.3, 0, 0.3]`・姿勢xyzw
`[1, 0, 0, 0]`から起動する。IK要求は同時に1件に制限し、遅い応答の蓄積を防ぐ。

既存の二腕launchを終了してから、ターミナルで次を実行する。
Quest入力publisherは従来通り別ターミナルで起動する。

```bash
source /opt/ros/jazzy/setup.bash
source ~/rebot/ros2_ws/install/setup.bash
ros2 launch quest_rebot_bridge dual_arm_rehearsal.launch.py
```

正面に向けて握ると、位置はその場を基準に、姿勢はコントローラーに合わせて
速度制限内で追従する。離すと保持、A/Xでホーム、トリガーで開閉。
再センタリングでQuestの座標系が変わった場合は再較正する。
可動域外の姿勢・位置ではIKが拒否し、その場を保持するため、全方向・全位置での
完全な追従を保証するものではない。

回帰検証：`test/test_orientation.py`と`tools/check_dual_arm_tracking.py`。
後者は別ROS_DOMAIN_IDで`dual_arm_rehearsal.launch.py rviz:=false`を起動し、
同じドメインから実行する（Quest入力publisherは同時起動しない）。
左右逆向きのXYZ各軸±12°・並進・クラッチ解除・握り直し・入力停止・ホームを確認する。

### ロール操作をジョイスティックへ変更（2026-09-17）

左手のみ姿勢がおかしくなる事象を較正・回転delta双方の数式検証で切り分けた結果
（`[motion]`ログの`rot_deg_quest`/`rot_deg_base`は角度が座標変換で不変なため
一致して当然で無意味、`axis_quest`/`axis_base`も"absolute"モードでは
較正回転と無関係に一致するため同様に無意味と判明）、コントローラー姿勢から
ロールを抽出する方式自体を疑い、joint6の役割をFKで直接検証した:

```
joint6のみ+0.3rad動かす → end_linkの位置変化ゼロ、姿勢変化はローカルX軸
まわり0.3rad(17.19°)の回転のみ（他2軸・位置は無変化）
```

→ **joint6 = エンドエフェクタのローカルX軸まわりの回転（ロール）そのもの**、
アーム姿勢に依らず常に成立、と確認。この事実を踏まえ、コントローラー姿勢
からロールを推定するのをやめ、**右スティックの左右傾きでロールを直接
レート制御**する方式に変更した（`bridge_node.py`）。傾きに比例して
`roll_offset_rad`を積算し、`compute_target()`の結果へローカルX軸回転として
後から合成する。スティック値は較正・controller_to_tool双方と無関係なため、
左右コントローラー間のデータ規約差があっても影響を受けない。

- `roll_rate_deg_s`(既定60、`max_angular_velocity_radps`のデフォルト上限
  約57°/sより少し上だが安全クランプで自然に頭打ちになる)、
  `roll_thumbstick_deadzone`(既定0.15)
- squeeze握り直しでロール角はリセットされない(保持)。ノード再起動時のみ
  既定の**0°**に戻る。**重要**: 最初`roll_offset_rad`の初期値を90°にしたところ
  `tools/check_dual_arm_tracking.py`が全軸で約90°ズレて失敗した。原因は
  `orientation_mode=absolute`では目標姿勢が`controller_pose.orientation ⊗
  controller_to_tool`そのもので「基準となる姿勢」という概念がそもそも無く、
  非ゼロの初期値を入れると毎フレーム勝手にその分だけズレ続けてしまうため。
  `home_orientation`(ホームリセット専用、下記)とは完全に独立した別概念であり、
  「同じ90°だから片方に合わせて良い」という思い込みが誤りだった。統合テストで
  発見・修正済み。
- `home_orientation`は、従来値`[1,0,0,0]`(ローカルX軸180°)から**さらに
  ローカルX軸+90°回転**した`[-0.707107, 0, 0, 0.707107]`に変更
  （グリッパー形状が重力に対して90°の方が相性が良いとのユーザー判断、
  回転行列合成で計算・検証済み）。ホームリセット(A/Xボタン)専用の値で、
  ロールのジョイスティック操作(初期値0°、上記)とは無関係。
  `tools/check_dual_arm_tracking.py`のホーム検証値も合わせて更新済み。
- `[heartbeat]`ログに`roll_offset_deg`を追加済み。
- `tools/check_dual_arm_tracking.py`は`thumbstick`データを送らないため、
  デコード側(`controller_data_codec.py`)を`.get()`で安全化(欠損時0.0扱い)。

**さらに2026-09-17内で発見**: 上記の実装（ジョイスティックのロールを
`proposed.orientation`に**加算**する`_apply_roll`）は、クエストの姿勢が
持つロール成分をそのまま残してしまい、「クエストからは指す方向(2自由度)
だけ使い、ロールは完全にジョイスティック由来にしたい」というユーザーの
意図と違っていた。**`_apply_roll`を`_pointing_pose`に置き換え**、
クラッチが計算した目標姿勢から**指す方向(forward、ローカルX軸)だけ抽出し、
ロール成分を完全に捨てて`up_hint=(0,0,1)`基準でゼロから再構成**した上で
ジョイスティックのロールを合成する方式に変更。同じ方向を指す2つの入力
（ロール成分だけ違う）が`_pointing_pose`後に完全一致することをテストで確認
(`test_pointing_pose_discards_controllers_own_roll`)。`test/test_orientation.py`
は7件PASS。

**次回やること**:
1. ~~`tools/check_dual_arm_tracking.py`の見直み。~~ 2026-09-26のエンドエフェクタ
   姿勢固定化（下記）で対応済み: 「姿勢が反映されないことを確認する」方向に
   書き換えた。ジョイスティックのロール経路自体はまだこのスクリプトでは
   未検証（thumbstickデータを送っていない）。
2. 上記修正を踏まえてQuest実接続で再検証（キャリブレーションのやり直しを
   含む）。

### エンドエフェクタ姿勢を常に鉛直下向きに固定（2026-09-26）

方針変更: コントローラーの向き（指す方向2自由度）でエンドエフェクタの姿勢を
追従させる従来方式をやめ、**姿勢の指す方向(pitch/yaw)を常に鉛直下向き
（base_linkの−Z）に固定**した。コントローラーをどう構えても、指す方向は
変化しない。

- 位置（3自由度、`compute_target()`）とロール（1自由度、右スティックで
  `joint6`のみを回す既存の経路）は変更なし。ロールは今まで通り
  クラッチのengage/release間で保持される。
- `bridge_node.py`の`_pointing_pose()`を変更: `compute_target()`が返す
  `proposed.orientation`（指す方向・ロールとも）を完全に無視し、固定の
  `_DOWN_POINTING_QUAT`（up_hint=(0,0,1)基準、モジュール内で1回だけ計算）に
  スティックのロールオフセットを合成するだけにした。
- この結果、`orientation_mode`（absolute/relative）・`orientation_source`
  （grip/aim）・`controller_to_tool_xyzw`は最終的なコマンド姿勢には
  一切影響しなくなった（`Clutch`/`controller_sample`自体は変更していないため
  内部的には引き続き計算されるが、出力は`_pointing_pose`が握りつぶす）。
  `[motion]`ログの生コントローラー回転量診断にのみ意味が残る。
  `dual_arm_rehearsal.launch.py`のこれらのパラメータもコメントで明記した
  うえで値自体は変更していない。
- ホームリセット（A/Xボタン、`home_position`/`home_orientation`）は
  `_pointing_pose`を経由しない別経路のため無変更。
- 回帰検証: `test/test_orientation.py`の`_pointing_pose`関連2件を、
  「入力姿勢が何であれ常に鉛直下向きになる」「ロールは固定下向き軸まわりの
  回転として効く」ことを確認する内容に書き換え（7件PASS）。
  `tools/check_dual_arm_tracking.py`は、コントローラーをXYZ各軸に±12°回転
  させても実際のEE姿勢は`_DOWN_POINTING_QUAT`のまま変化しないことを確認する
  向きに書き換えた（旧版はコントローラー回転にEEが追従することを確認していた）。

### 鉛直下向き固定とhome_position/home_orientationの不整合を修正（2026-09-26）

実機Quest + `dual_arm_rehearsal.launch.py`でテレオペを試したところ、クラッチを
握った直後から位置が一切追従しなくなる不具合が発生。`/left_arm/target_pose`は
握った瞬間の値のまま固まり、`[heartbeat]`ログには
`move_to_pose_ik rejected: IK did not converge (err=0.001236)`が出続けていた。

**原因（IKソルバーを直接叩いて特定）**: `fake_ik_driver.py`が使う数値IK
（減衰最小二乗法、収束閾値`1e-4`）で、位置`(0.3, 0, z)`を固定して`z`を振ると:

```
z=0.60〜0.20: 失敗（誤差はzが上がるほど滑らかに増加、0.006〜0.39）
z=0.15以下 : 成功（誤差 ~1e-4）
```

**「鉛直下向き」姿勢は、このアーム(B601-DM+グリッパー)の関節構造上、
z≲0.15〜0.20より高い位置ではそもそも実現不可能**（50回のランダム初期値
再試行でも一切解が見つからず、誤差がzに対して単調増加＝解なしを裏付ける
パターン）。バグではなく純粋な可到達域の限界。

ところが`home_position`（既定`(0.3,0,0.3)`）と`home_orientation`
（既定`identity`、`dual_arm_rehearsal.launch.py`では別途
`[-0.707107,0,0,0.707107]`）は、どちらもこの新しい「常に鉛直下向き」方針とは
無関係な旧来の値のまま残っていた。その結果、クラッチをホーム姿勢で握った
瞬間、目標姿勢が即座に`_DOWN_POINTING_QUAT`（水平寄りのhome姿勢から
約120°の姿勢差）へジャンプし、safety_limitsで角速度制限された小刻みな
回転ステップの多くがz=0.3付近の到達不能領域を通ることになり、IKが
軒並みrejectされて位置が固まって見えていた。

**方針（ユーザー要望）**: 「起動時のホーム姿勢そのものを鉛直下向きにし、
クラッチを握った瞬間の場所からの相対移動だけで動かす」。この場合
home_orientationとテレオペ中の固定姿勢が最初から一致するため、握った瞬間の
再姿勢合わせ自体が不要になる。

**修正**:
- `bridge_node.py`: `home_position`既定値を`(0.3,0,0.3)`→`(0.3,0,0.10)`
  （IK検証済みの可到達域）、`home_orientation`既定値を`_DOWN_POINTING_QUAT`
  そのものに変更。`workspace_max`のz上限を`0.6`→`0.15`に変更
  （それより上は元々IKが拒否するだけなので、ワークスペース側でも整合させた）。
- `dual_arm_rehearsal.launch.py`: `home_orientation`を同じ
  `_DOWN_POINTING_QUAT`に更新。シミュレータの`initial_joints`も
  新しいhome姿勢に対応する関節角をIKで計算し直して起動時から
  home姿勢に一致するようにした（起動直後の握りでも再姿勢合わせが発生しない）。
- `tools/check_dual_arm_tracking.py`: `home`を`_DOWN_POINTING_QUAT`から
  導出するように変更（値の二重管理をやめた）、ホーム到達判定の期待位置を
  `(0.3,0,0.10)`に更新。

**未検証**: 実機B601-DMでの再検証（関節限界・URDFがシミュレーションと
完全に一致する前提が正しいか含む）。`home_position`/`workspace_max`は
シミュレーションモデルのIK検証のみに基づく値である旨、`bridge_node.py`の
起動時警告ログにも明記した。

実機Quest + 大学Wi-Fi + テザリングでのテレオペ動作も確認済み。ラグを感じた
原因は大学Wi-Fiの輻輳と判断（同一ネットワーク内でもping 9〜312msと大きく
ジッターしていた）。切り分け用に`controller_raw_publisher`が元々payloadへ
埋め込んでいた`timestamp`を`bridge_node.py`側で使うようにし、
`[heartbeat]`ログに`pipeline_latency_ms`（CloudXR SDKがサンプルを返して
からbridge_nodeが処理するまでの、ROS側パイプラインだけの遅延）を追加した。
Wi-Fi/CloudXR側の遅延はこの値には現れない（SDK呼び出しが返った後に
打刻するため）ため、Quest本体からPCまでの区間の遅延を分離して見たい場合は
別途ネットワーク側の計測が必要。

### 実機Quest + dual_arm_rehearsal での動作確認手順（2026-09-26）

**実行方法**（ターミナル2つ。`bridge.launch.py`/`moveit_rehearsal.launch.py`
は不要 -- `dual_arm_rehearsal.launch.py`単体でシミュレータ・RViz・
bridge_node（左右分）まで全部起動する）:

```bash
# ターミナル1: シミュレータ本体 + RViz
source /opt/ros/jazzy/setup.bash
source ~/rebot/ros2_ws/install/setup.bash
ros2 launch quest_rebot_bridge dual_arm_rehearsal.launch.py \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_dual_arm.log
```

```bash
# ターミナル2: controller_raw_publisher（isaacteleop venv、Questと接続）
source /opt/ros/jazzy/setup.bash
source ~/rebot/.venv-teleop-ros2/bin/activate
cd ~/rebot/ros2_ws/src/quest_rebot_bridge
python3 -m quest_rebot_bridge.controller_raw_publisher \
  2>&1 | tee ~/rebot/logs/$(date +%H%M%S)_controller_raw.log
```

同じ`ROS_DOMAIN_ID`を両ターミナルで使うこと（未設定なら両方未設定のままでも可、
どちらか一方だけ`export ROS_DOMAIN_ID=...`していると互いに見えなくなる）。

**CloudXRクライアント（Quest側）の接続設定**: 自動探索（mDNS的な近隣デバイス
発見）は今回の大学Wi-Fiでは機能しなかった。**Server IPとPortを両方手動入力**
する必要がある:
- Port は固定で `48322`（`~/.cloudxr/logs/wss.*.log`の
  `WSS proxy listening on port ...`で確認可能）。Portを空欄のままだと
  既定の別ポートを試みて繋がらない。
- Server IP はPC側で`ip -4 addr show <wifi_if>`（例:`wlP9s9`）で確認した
  現在のアドレス。ネットワークを切り替えるたびに変わるので都度確認し直す。

**ネットワークの注意点（ラグの原因と対策）**: 大学Wi-Fiのような混雑した
共有ネットワークでは、同一ネットワーク内の機器同士でもping遅延が実測で
9〜312msまで跳ね上がることがあり、これがテレオペの「常に一定の遅延がある」
という体感の主因になり得る（CloudXRなどのリアルタイムストリーミング系の
実装はジッターバッファでこれを吸収するため、ジッターが"一定のやや大きい遅延"
に変換されて感じられやすい）。

対策として、QuestとPCを**専用の空いているネットワーク**に繋ぐのが最も効く。
今回はスマホのテザリング（モバイルホットスポット）に両方繋いで解決した。
Quest⇔PC間の通信はインターネットに出て行かず、映像ストリーミングもしていない
（`controller_raw`モードは姿勢・ボタン情報のみ）ため、テザリングの携帯データ
消費量はほぼ気にしなくてよい。

**注意**: このPCのようにWi-Fiカードが1枚しかない環境では、テザリング等の
別ネットワークに繋ぎ替えている間、元のネットワーク（大学Wi-Fi）からは
完全に切断される。このPC自体をWi-Fiアクセスポイント化する案も検討したが、
その間このPCの他の通信（SSH、リモート作業ツールなど、大学Wi-Fi経由の
インターネット接続に依存するもの）も道連れで切断されるリスクがあるため
見送り、スマホのテザリングを採用した。

### 死んだ姿勢追従コードの削除（2026-09-26）

姿勢を鉛直下向きに固定する方針への変更（上記）により、`Clutch`の
`orientation_mode`（absolute/relative）・`controller_to_tool`軸マッピング・
`rotation_scale`は、`_pointing_pose()`が`compute_target()`の姿勢出力を
常に握りつぶすため、二度と外部から観測できない完全なデッドコードになって
いた。以下の通り削除・整理した:

- `clutch.py`: `Clutch`を位置追従のみに簡素化。`orientation_mode`/
  `controller_to_tool`引数と、姿勢のabsolute/relative計算ロジックを削除。
  `compute_target()`は`rotation_scale`引数も廃止。
- `bridge_node.py`: `orientation_mode`/`controller_to_tool_xyzw`パラメータの
  宣言と`rotation_scale`パラメータを削除。`orientation_source`（grip/aim）は
  `[motion]`診断ログ用の値としてのみ引き続き有効（実際のコマンド姿勢には
  無関係、というコメントを追加）。
- `dual_arm_rehearsal.launch.py`: 上記2パラメータの上書き設定を削除
  （残したままだと`bridge_node`が宣言していないパラメータとしてlaunch時に
  エラーになるため）。
- `test/test_orientation.py`: `Clutch`のabsolute/relative姿勢を検証していた
  3件のテストを削除し、位置追従のみを検証する1件に置き換え（計5件PASS）。
