# COBOTTA トレーニング教材

人協働マニピュレーションシステムを使って、COBOTTA に作業を実装する講習用のワークスペースです。

## 構成

```
cobotta_training/
├── README.md
├── answer/
│   └── CollaborationUserDefineModule.py   作業1・2・3の解答
└── src/
    ├── collaboration_manipulation/        講習で使うノード群
    │   ├── collaboration_manipulation_module/   ロボット制御本体（受講者が実装する）
    │   ├── management_system/                   上位アプリ（作業指令を出す）
    │   ├── workpieces_detection_subsystem/      ワーク検出（仮実装）
    │   ├── collaboration_manipulation_message/  メッセージ・サービス定義
    │   └── （その他のサブシステム：講習では未使用）
    └── denso_robot_ros2/                  COBOTTA ドライバ（git 管理外。下の手順で取得）
```

受講者が書き換えるファイル：
`src/collaboration_manipulation/collaboration_manipulation_module/collaboration_manipulation_module/CollaborationUserDefineModule.py`
の `operating_work()` の中の作業1・作業2（`#TODO` の部分）。作業3は逆運動学の紹介用として実装済み。

## 動作環境

- Ubuntu 22.04
- ROS 2 Humble
- COBOTTA（RC8 コントローラ）

## 環境構築

### 1. 依存パッケージのインストール

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-smach ros-humble-smach-ros python3-rospkg python3-yaml
```

### 2. 教材の取得

```bash
cd ~
git clone git@github.com:rsdlab/cobotta_training.git
```

### 3. COBOTTA ドライバ（denso_robot_ros2）の取得

教材で動作確認したバージョン（v1.2.0 ブランチのコミット `67847ab`）に固定して取得する。

```bash
cd ~/cobotta_training/src
git clone -b v1.2.0 https://github.com/haraisao/denso_robot_ros2.git
```

### 4. 残りの依存関係とビルド

```bash
cd ~/cobotta_training
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> 他のワークスペース（例：`~/colcon_ws`）を `~/.bashrc` で読み込んでいると、
> そちらのパッケージが混ざって動作することがある。教材の動作確認は、
> 他のワークスペースを読み込まない端末で行うこと。

## 起動手順

端末を4つ開き、それぞれで `source ~/cobotta_training/install/setup.bash` してから、この順に起動する。

```bash
# 1. COBOTTA ドライバ（ip_address はCOBOTTAの IP に合わせる）
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py \
    model:=cobotta sim:=false ip_address:=192.168.XXX.XXX send_format:=0 recv_format:=66

# 2. ワーク検出
ros2 run workpieces_detection_subsystem WorkpiecesDetectionNode

# 3. 人協働マニピュレーションモジュール（初期化で COBOTTA が初期姿勢へ動く）
ros2 run collaboration_manipulation_module CollaborationManipulationModule

# 4. 上位アプリ（作業指令を送る）
ros2 run management_system ManagementSystemNode 1
```

- `recv_format:=66` にするとモータ電流が配信され、作業2の「引っ張り検知」が使える。
  `recv_format:=2` の場合は合図トピック `/handover_ok` だけで受け渡しを判定する。
- 4 はマニピュレーションモジュールの初期化が終わり、待機状態になってから実行する。

### 上位アプリの引数

| 引数 | 動き |
|---|---|
| なし | 作業1 → 2 → 3 を順に実行（失敗したら中断） |
| `1` / `2` / `3` | 指定した作業だけ実行 |
| `halt` | マニピュレーションモジュールに終了指令を送る |

## 作業の内容

| 作業 | 内容 | 講習での扱い |
|---|---|---|
| 1 ピック＆プレース | 机の上のワークをつかみ、トレイへ置いて初期姿勢に戻る | 課題 |
| 2 人への受け渡し | ワークを受け渡し位置へ低速で運び、人が受け取ったら離す。10秒以内に受け取られなければ元の位置へ戻して置く | 課題 |
| 3 逆運動学動作 | 手先の位置を指定して上・横へ 3cm 動かし、元の位置へ戻る | 紹介（実装済み） |

作業1・2では、準備中にワーク検出を行う。ワークがない場合は動作せず、作業結果を「失敗」として通知する。

## 動作確認に使う操作

```bash
# ワークなしを模擬する（作業1・2が動かないことを確認）
ros2 run workpieces_detection_subsystem WorkpiecesDetectionNode --ros-args -p work_present:=false

# 作業2で「受け取った」合図を送る
ros2 topic pub --once /handover_ok std_msgs/msg/Empty {}

# 手先の現在位置を確認する（pose_manip() に渡す座標を調べる）
ros2 run tf2_ros tf2_echo base_link tool

# 関節角度を確認する（joint_manip() に渡す値を調べる。値は rad なので deg に変換する）
ros2 topic echo /joint_states --once
```

## 解答

`answer/CollaborationUserDefineModule.py` が作業1・2・3の解答。
関節角度は解答作成時の環境でティーチングした値なので、設置位置が違う場合は取り直すこと。
