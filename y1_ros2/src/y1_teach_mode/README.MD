# y1_teach_mode 使用说明（初稿）

本目录为 **ROS 2（rclpy）** 示教相关脚本，依赖已编译的 `y1_msg` 与工作空间环境。使用前请在终端执行：

```bash
source /path/to/y1_ros2/install/setup.bash
```

## 一、`normal_mode/` — normal模式（底层有插值平滑算法）下的轨迹录制与回放

适用于 **关节位置控制（`ArmJointPositionControl`）** 或 **末端位姿控制（`ArmEndPoseControl`）** 的轨迹文件（如 25 Hz 采样）。

| 脚本 | 作用简述 |
|------|-----------|
| `record_trajectory.py` | 订阅关节状态，按固定频率将 `position` / `velocity` / `effort` / `end_pose` 写入 jsonl |
| `joint_position_play_trajectory.py` | 从 jsonl 按频率回放 **关节位置**；根据首行 `position` 长度 **6 或 7** 决定是否带 `gripper_stroke`（第 7 维） |
| `end_pose_play_trajectory.py` | 从 jsonl 按频率回放 **末端位姿**；同样根据首行 `position` 为 **6 或 7** 决定是否下发夹爪行程 |

**运行示例：**

**A 录制轨迹**

启动机械臂：

```bash
source y1_ros2/install/setup.bash
ros2 launch y1_controller one_master.launch.py
```

录制轨迹：

```bash
cd /path/to/y1_teach_mode/normal_mode
python3 record_trajectory.py
```

**B 播放轨迹**（以下脚本不可同时运行）

重新以normal模式启动机械臂：

```bash
source y1_ros2/install/setup.bash
ros2 launch y1_controller single_arm_control.launch.py
```

按关节控制播放：

```bash
cd /path/to/y1_teach_mode/normal_mode
python3 joint_position_play_trajectory.py
```

按末端位姿控制播放：

```bash
cd /path/to/y1_teach_mode/normal_mode
python3 end_pose_play_trajectory.py
```

**常用 ROS 参数（各脚本略有不同，以 `declare_parameter` 为准）：**

- `joint_state_topic`：录制时订阅的关节状态话题  
- `output_jsonl` / `trajectory_jsonl`：输出或输入的 jsonl 路径  
- `joint_position_control_topic` / `end_pose_control_topic`：回放时发布的话题  
- `playback_rate_hz` / `record_rate_hz`：频率  

**数据约定：** jsonl 每行一条 JSON，至少包含 `position`、`end_pose` 等字段；回放脚本根据 **首条** `position` 元素个数判断 **6（仅机械臂）** 或 **7（第 7 维为夹爪行程）**。

---

## 二、`teach_trajectory/` — 示教轨迹（高频等）

与 `normal_mode` 类似，但默认面向 **更高采样率** 的轨迹文件（如 400 Hz），且 **关节流回放** 使用 `ArmJointState` 发布到从臂跟随话题（与 `normal_mode` 下关节回放消息类型不同）。

| 脚本 | 作用简述 |
|------|-----------|
| `record_trajectory.py` | 高频录制到 jsonl（默认文件名/频率与 `normal_mode` 不同，见脚本内参数） |
| `joint_position_play_trajectory.py` | 按首行 `position` **长度**截取整段下发（可变维关节数组） |
| `end_pose_play_trajectory.py` | 末端轨迹 + 与 `normal_mode` 相同的 **6/7 维 position → 夹爪** 逻辑 |

**运行示例：**

**A 录制轨迹**

启动机械臂：

```bash
source y1_ros2/install/setup.bash
ros2 launch y1_controller one_master.launch.py
```

录制轨迹（高频 jsonl，默认参数见脚本内）：

```bash
cd /path/to/y1_teach_mode/teach_trajectory
python3 record_trajectory.py
```

**B 播放轨迹**（以下脚本不可同时运行）

关节流回放依赖从臂订阅 `ArmJointState`，请先启动从臂：

```bash
source y1_ros2/install/setup.bash
ros2 launch y1_controller one_slave.launch.py
```

按关节流控制播放：

```bash
cd /path/to/y1_teach_mode/teach_trajectory
python3 joint_position_play_trajectory.py
```

按末端位姿控制播放：

```bash
cd /path/to/y1_teach_mode/teach_trajectory
python3 end_pose_play_trajectory.py
```

**注意：** 实际话题名需与当前 `y1_controller` 启动配置（yaml/launch）一致；主从同时在线时也可改用 `one_master_slave.launch.py`，按现场修改参数。

---

## 三、`teach_pose/` — 离散位姿示教

按 **关键点** 录制与回放（非连续轨迹时间轴），适合逐步示教多个姿态。

| 脚本 | 作用简述 |
|------|-----------|
| `record_pos.py` | 交互式：每次回车记录一帧到 jsonl（含 `position`、`end_pose` 等） |
| `joint_position_play_pos.py` | 依次下发 **关节位置** 控制，等待当前关节接近目标后再发下一点；支持 **6/7 维 position** 与 `gripper_stroke` |
| `end_pose_play_pos.py` | 依次下发 **末端位姿**，等待反馈接近目标；**6/7 维 position** 与夹爪逻辑同轨迹回放 |

**运行示例：**

**A 录制离散位姿**

启动机械臂：

```bash
source y1_ros2/install/setup.bash
ros2 launch y1_controller one_master.launch.py
```

录制位姿（终端内按提示回车记录、`q` 退出）：

```bash
cd /path/to/y1_teach_mode/teach_pose
python3 record_pos.py
```

**B 播放位姿**（以下脚本不可同时运行）

脚本默认订阅/发布 `/y1/arm_joint_state` 等话题，请重新以 normal 模式启动机械臂（示例）：

```bash
source y1_ros2/install/setup.bash
ros2 launch y1_controller single_arm_control.launch.py
```

按关节控制逐点播放：

```bash
cd /path/to/y1_teach_mode/teach_pose
python3 joint_position_play_pos.py
```

按末端位姿逐点播放：

```bash
cd /path/to/y1_teach_mode/teach_pose
python3 end_pose_play_pos.py
```

**常用参数：** `joint_state_topic`、`joint_position_control_topic` / `end_pose_control_topic`、`poses_jsonl`、`position_tolerance` / `pose_tolerance`、`sleep_after_reach_sec` 等。

---

## 四、通用说明

1. **依赖**：`rclpy`、`y1_msg`；需有对应话题上的 **发布方/订阅方**（如控制器节点）。  
2. **交互**：多数脚本含 `input()`，请在有标准输入的终端运行。  
3. **路径**：jsonl 默认多为相对路径 `data/xxx.jsonl`，若启动目录不对会找不到文件，建议改为绝对路径或通过参数指定。  
4. **与控制器配置对齐**：`y1_ros2/src/y1_controller/config/` 下 yaml 中的话题名需与脚本参数一致，否则无数据或无法控制。
