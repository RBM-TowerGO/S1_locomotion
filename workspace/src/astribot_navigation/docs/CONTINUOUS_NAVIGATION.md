# 真机导航

## 启动与 ROS 接口

在 S1B 的 `workspace/src/astribot_navigation` 目录启动导航。已有地图或配置参数仍传给这个入口。

```bash
./scripts/tg-navigation.sh --mode hardware
```

网页按需在 S1B 的另一个终端接入，默认监听本机 8767 端口。退出或重新启动网页后，可以继续查看原任务。网页的历史轨迹从该客户端接入时开始绘制。

```bash
./scripts/tg-navigation-web.sh --port 8767
```

真机与观察模式均使用 Domain 25。回放沿用 `tg-navigation.sh --mode mock --replay BAG`，可选网页使用 `tg-navigation-web.sh --replay`（Domain 125）。SSH 转发仍为 `ssh -N -L 8767:127.0.0.1:8767 S1B`。

在 S1B 的 Bash 中可以直接读取当前状态和接口定义：

```bash
source scripts/navigation_env.sh
ros2 interface show astribot_nav_interfaces/srv/NavigationCommand
ros2 topic echo /towergo/nav/task_state --qos-reliability best_effort --once
```

`/towergo/nav/command` 使用 `astribot_nav_interfaces/srv/NavigationCommand`。同一个调用方使用稳定的 `owner`（16–128 个字符）。`accepted` 表示监督节点已受理，拒绝原因在 `reason`。后续进展和最终结果通过 `/towergo/nav/task_state` 获取，类型为 `std_msgs/msg/String`，内容为 JSON。定位按 `localization_id` 对应 `localization.id/status`，运动按 `task_id` 对应 `task/phase/result`。运动的最终结果沿用停车确认。

| operation | 输入 |
|---|---|
| `localize` | `owner, x, y, yaw`，地图坐标（m、rad） |
| `goal` | 同上，加 `speed`（m/s） |
| `nudge` | `owner, x, y, yaw, speed`，表示机器人自身坐标系的前移、左移和转角增量 |
| `stop` | 只需 `operation`，取消当前操作 |

从源码部署时，在 S1B 的 `S1_locomotion` 仓库根目录编译接口包一次，两个启动脚本都会加载它。

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/runtime_ws/build_python:${PYTHONPATH:-}"
cd workspace/navigation_ws
colcon build --base-paths ../src/astribot_navigation/astribot_nav_interfaces --packages-select astribot_nav_interfaces --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
```

## 导航控制链路

控制链路改为 Nav2 的常规结构：`controller_server → /towergo/nav/cmd_vel_nav → 执行器 → SDK`。

- **执行器**直接订阅 `cmd_vel_nav`。收到非零速度时自动开始一个内部任务，硬限速取 `navigation.yaml` 的 `max_speed`；每个目标的速度由导航监督节点写入 RPP 的 `desired_linear_vel`。0.3 s 收不到速度指令就原地保持（目标结束、取消、控制器卡住都走这一条）；保持稳定之前不接受新的运动。
- **导航监督节点**负责传感器桥接、定位初始化与质量监测、提交和取消任务、停稳确认，独立于 HTTP 端口。
- **网页**调用监督节点的 ROS 服务，订阅状态、地图、路径和雷达话题，在自己的进程内生成显示叠加。
- **停止**：取消 Nav2 目标，同时通过 `executor_control` 发带外保持指令；执行器在 1 s 内忽略尚未停下的控制器输出。普通停止、到达、失败都只做原地保持，不再调用厂商整机停止。整机停止只用于不可恢复的硬故障，触发后需要重启会话。
- **运行监督**：传感器断流、map→base 变换超时或 AMCL 未激活、执行器故障或无控制权、Nav2 未激活，任一项持续 1 s 才取消目标；短暂抖动不再打断任务。

## 2026-11-20 收敛为单一决策链

此前规划、跟随、执行器、网页四层各自都能判定“不对”并自行停车、重转或重提目标。外层把内层的正常输出当成故障，每次修一处就多一道补丁。现在改为每层只有一个决策者，各层之间不再互相覆盖。

| 层 | 唯一决策者 | 删除的内容 |
|---|---|---|
| 规划 | Nav2 `SmacPlanner2D`（点机器人代价搜索，膨胀半径不小于外接半径） | ExactGoalLattice、omni/ackermann 基元、转弯半径、直线优先捷径、规划后整机复核抛异常 |
| 跟随 | Nav2 `RegulatedPurePursuitController`：路线在背后时原地转，否则前进并按曲率/障碍/接近减速，按整机轮廓预测碰撞 | RotationShim + DWB（441 条采样、6 个手调 critic），普通导航中的横移与倒车 |
| 重试 | 行为树：1 Hz 重规划；失败后清局部代价地图、等待 2 s，最多 6 轮，用尽后 abort | 无限重试加 1 s 延迟 |
| 执行器 | 按实际周期积分（单周期上限 50 ms），轴停止或反向时撤回该轴领先量 | 10 ms 积分上限（实际速度只剩 50–83%）、纯转向前的 0.15 s 冻结（turn_hold） |
| 任务监督 | 提交目标、显示诊断，并在持续硬故障时停车 | 暂停→等待 2 s→重新提交的状态机；定位质量导致任务取消；自动 5 帧搜索恢复；运行中的 no-motion 保鲜调用；协方差静止重评估 |

运行中只有以下硬条件会停车并结束任务（不会自动重提）：map→base TF 超时或 AMCL 未激活、传感器断流或 LIO 历史中断、SDK 反馈/控制权/执行器故障、Nav2 返回 abort、手动停止。

扫描与地图匹配质量、AMCL 协方差、amcl_pose 更新节奏改为只告警：定位初始化成功和提交目标前必须通过，运行中只显示。匹配失配会锁存，下一次提交前需要重新初始化。

新会话会接管旧会话（`real_navigation/takeover.py`）。同一用户、同一 ROS Domain 下再次运行 `tg-navigation.sh` 时：先向旧会话发 SIGTERM，让它走正常收尾（执行器停车并交还控制权）；40 s 内未退出，就按“执行器 SIGINT → 旧主进程 SIGKILL → 其余进程组 SIGINT/SIGKILL”的顺序清理。会话自有的残留进程（带 `__ns:=/towergo/nav`、`navigation_fastlio`、`navigation_dual_adapter`、`real_navigation.executor` 等标记）也一并清理；Livox 驱动等共享进程不受影响。锁文件 `/tmp/towergo-navigation-<uid>-<domain>.lock` 记录当前会话的 PID 和目录。正常 Ctrl+C 退出时，执行器最多等 8 s，其余进程各等 5 s，超时后升级为 SIGTERM/SIGKILL。

到点容差从 5 cm / 3° 放宽到 0.10 m / 0.10 rad，与 AMCL 自身的抖动量级相匹配。小位移微调（Nudge）的横移能力与容差规则不变。ExactGoalLattice、StopTurn 等插件仍然编译，但不再被选用。

## 2026-11-19 全向导航与网页微调（历史）

普通导航由 Smac 的全向运动基元规划路线，DWB 根据当前机身轮廓、前后雷达和地图选择速度。行为树每秒检查路线，有效路线持续跟随，路线无效时重新规划，最长每 30 秒更新一次。

2026-11-20 第二次调整：关闭 lattice 的 `allow_reverse_expansion`。会话 navigation_20261120_000733 的 6 条全局路径都以 0.7 m 倒车钩子开头（首段朝向与车头相差约 180°，0.7 m 内路径朝向转约 100°），RotationShim 每次重规划都朝新的起点朝向重转，机器人只在约 1.6 m 范围内打转。全向基元本身能横移，不再需要倒车起步。同次加入静止时的 AMCL 无运动更新请求：底盘停住后 AMCL 因 update_min_d/update_min_a 不再发布位姿，`amcl_update_expected_ok` 变假，会话以"等待扫描与定位变换恢复"反复暂停；现在静止且传感器健康时每 0.8 秒请求一次 `/towergo/nav/request_nomotion_update`，不伪造位姿。

全局 costmap 只用静态地图和膨胀层，前后雷达的实时障碍只进滚动的局部 costmap。原因：障碍点云去掉了地面点，空地上没有射线能清除已标记格子，固定的全局地图里误检点长期残留，使路线被判无效并反复绕路。Smac cost_penalty 由 3.0 降为 2.0。代价：全局规划不再主动绕开地图外的新障碍，由 DWB 在局部 costmap 内避让或停车；若长期堵住，需更新地图。手动微调仍跟随指定的直线或原地转角，保持最初的里程计目标。

网页速度顺序为前进、横移、转向。平移合速度受页面设定限制，横移上限 0.15 m/s，倒车上限 0.10 m/s（2026-11-20 由 0.05/0.03 提高；回退恢复仍为 0.03 m/s）。FollowPath 为 RotationShim 包 DWB：路线方向与车头偏差 >0.6 rad 时先以 0.3 rad/s 原地转到 0.15 rad 内，再交给 DWB。原因：会话 navigation_20261119_235355_0549a9 起步背对路线约 150°，DWB 在慢横移、慢倒车和转向间来回切换，95 s 未离开 6 cm 范围后报 Failed to make progress。诊断录包新增局部 costmap、机身轮廓、全局路径和前后障碍点云。普通目标容差为 5 cm、3°，小位移微调使用相应的更小容差。

## 2026-10-29 连续曲线导航

用户已明确允许边走边转，以连续移动优先，替代原停车转向策略。真机/录包入口当前选择 ExactGoalLattice + Nav2 RegulatedPurePursuitController + SimpleGoalChecker。保持静态地图、轮廓/padding、定位模型和质量判据，默认0.05 m/s、原有速度上限不变。

行为树 PipelineSequence 中规划每秒更新，与 FollowPath 并行；不再串行完成规划后才能跟随，不执行原固定1秒Wait恢复。RPP接收新路径不重置停车转角状态。普通转弯按曲率与障碍代价减速跟随，lookahead=0.35m，原地对准阈值0.7rad，保留必要初始/终点朝向校准。曲率碰撞预测保持开启，不能在无可行路径时强行保持非零。

这不保证任意条件全程非零：碰撞、定位/反馈/许可失效、停止请求、终点以及必要的朝向校准仍可能停车。没有重用过期非零指令来跨过真实故障。当前仍仅用静态地图避障，实时扫描用于定位/预览，不自动发现地图外新障碍。

## 隔离验证

localhost Domain128，实际Nav2规划器/控制器，理想底盘，无SDK：相同实际生成参数，9次规划/路径更新，运行检查369个采样零线速度0次，532个仿真采样同时具有线/角速度，取消后正常停止。仅短段验证，不是完整真机路线、物理制动或硬件跟踪精度验收。证据 workspace/data/reports/curved_replan_20261029/。

重启现有tg-navigation.sh --mode hardware --map <原地图>入口，Ctrl+F5刷新并重新初始化加载；没有自动启动SDK或发送真实运动目标。此前STOP_TURN_NAVIGATION.md保留为原方案历史说明，其插件不再是当前真机默认。

最终软件回归252 passed（33.92 s），硬件依赖 --check 通过（不加载SDK）。

## 2026-10-29 23时：重规划应答超时

真实会话 navigation_20261029_225952_f3671e 第一个目标成功，第二个目标失败。Nav2明确记录 compute_path_to_pose 请求确认超时，随后行为树取消FollowPath；不是本次碰撞触发。默认default_server_timeout=20毫秒过紧，改为1000毫秒；bt_loop_duration由10改为50毫秒，降低100Hz行为树调度开销。超时是动作协议确认等待，不是原始传感器或SDK看门狗；规划期间并行跟随不变。

动作失败页面读取本目标开始后的Nav2日志区间，明确区分规划请求应答超时/无进展/无可行轨迹，避免仅显示“Nav2任务结束”，也不拿上一任务错误误归因。停车确认仍保留SDK+LIO证据，增加pending字段和具体等待条件，完整记录stop_checks。不把请求停车等同物理已停。

## 2026-10-29 加大机身边界和转弯规划

用户报告贴障碰撞风险，现场日志持续RPP detected collision ahead并Controller patience exceeded。原inflation 0.20m小于旧轮廓内切半径0.35m，Nav2已报错。现规划/控制共用之前用户确认的固定机身包络：前0.50、后0.40、左右0.40m，每侧padding=0.05m，即碰撞包络约1.0×0.9m；inflation=0.65m（软代价范围，并非整圈硬禁行）。

规划基元改为1m turning radius ackermann（原0.5m diff），关闭平滑以保留基元形态；新增prefer_straight_route开关，真机false，防止旧直线捷径绕过转弯基元。未改离线通用仿真默认开关true。规划后以完整矩形及内部占用格逐段重查，包括精确终点拼接；终点朝向调整仍可原地转动且接受碰撞检查。RPP前视0.5m。规划基元半径不是任意闭环偏差下的实测转弯半径保证。

真实地图只规划：当前定位(-4.9185,32.4717,-1.1420)不满足扩大后边界，目标(-5.4923,29.5936,-2.1200)满足；规划器报起点在lethal空间。附近静态余量满足的参考点(-4.8685,32.5717,-1.1420)仍无有效1m半径路径。不能用该参考点替换真实定位、不能据此自动移车。用户需现场核对定位与机身位置，在宽敞处重新初始化。新增提交目标前实际起点余量检查，避免不满足边界仍提交运动。证据wide_turn_actual_map_20261029、wide_turn_valid_reference_20261029。

隔离开放场景真实Nav2/RPP：9次路径更新、361运行采样中零线速度0次、取消停车通过，包含532个v/w同时非零的仿真采样；未连接SDK。证据wide_turn_curved_20261029。226项回归通过，另新增起点阻挡保护测试通过，C++几何CTest1/1及插件重新编译安装成功。真实路线仍需现场验收。

最新：2026-10-29 起步复核修复及安全回退见 RECOVERY_BACKUP.md。膨胀默认0.85m（此前0.65m未覆盖外接角），完整边界和1m规划基元保留。导航失败可在确认停稳、后方静态与实时云检查通过后回退0.15m再规划，最多两次；实际接触/数据失效不盲目倒车。

## 2026-11-05 原地转向回退规划

用户要求窄处可原地调向。每次重规划先用原 ackermann 弧线基元（GridBased，行为不变）；仅当无弧线路径时，Fallback 再调用 InPlaceTurn（同半径 diff 基元 = 同组弧线 + ±22.5° 原地转），死胡同掉头、过窄转角由此可解。两者参数相同，均经 ExactGoalLattice 完整轮廓重查。

- 不直接把主规划换成 diff：Smac 只在 16 个航向格点检查原地转的碰撞，格点之间机身角可扫进墙；完整重查会拒绝这种路径，紧 L 角因此从“弧线可解”退化为无路径。
- InPlaceTurn 单独一个 planner_server（命名空间 /towergo/nav/inplace，独立 lifecycle manager 和同参数静态全局 costmap），由 navigation.xml 的 server_name 指向。原因是 Humble Smac 的 motion_table/距离启发表是进程级 static，两个 lattice 插件同进程会互相覆盖（实测死胡同同进程回退失败、分进程成功）。
- RPP 控制器、执行器、速度/领先预算均未改动；原地转由 RPP rotate_to_heading 执行（0.3 rad/s 上限）。

隔离 Domain128 理想底盘、真实 Smac+RPP、无 SDK（证据 inplace_turn_fallback_20261105）：死胡同 1.5 m 走廊掉头，原方案无路径→回退方案成功（InPlaceTurn 8/90 次）；1.2 m 紧 L 角，成功，InPlaceTurn 4/124 次；开阔场景仅用 GridBased，与原方案一致。三者机身轮廓与障碍格零交叠，终点误差 ≤0.15 m / 0.075 rad。真实 launch 参数下 inplace 进程加载 diff 基元、订阅 /towergo/nav/map 并与主栈一样等待 tg_nav_map TF。未做完整 bt_navigator 联调和真机验收。

已知限制（原有）：ExactGoalLattice 以起点→终点直线方向作为到达朝向，窄死胡同里斜向目标可能因到达姿态放不下而无路径。
