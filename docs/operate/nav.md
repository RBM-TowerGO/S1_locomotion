# S1B 底盘导航操作速查

## 在 Mac 上启动真机导航

**终端 1 · 启动导航。** 在 Mac 上复制这一行并保持终端打开。默认地图为 `manual_edit_20261105_speckle_fill/map.yaml`。要沿用自定义地图或配置，在同一条远端命令的 `--mode hardware` 后加 `--map` 或 `--config` 及对应的 S1B 绝对路径。已有导航会话时，直接从终端 2 接入；重复执行终端 1 的命令会接管旧会话。

```bash
ssh -t S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation.sh --mode hardware'
```

**终端 2 · 打开网页（可选）。** 在 Mac 新开一个终端，复制下面这行。它同时在 S1B 启动网页、把网页端口转发到本机。终端显示网页地址后，打开 [S1B 导航页面](http://127.0.0.1:8767)。

```bash
ssh -t -o ExitOnForwardFailure=yes -L 127.0.0.1:8767:127.0.0.1:8767 S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation-web.sh --port 8767'
```

如果 S1B 上的网页进程已经在运行，终端 2 只需建立转发：

```bash
ssh -NT -o ExitOnForwardFailure=yes -L 127.0.0.1:8767:127.0.0.1:8767 S1B
```

### 在网页上定位、导航和停车

1. 首次定位时，让底盘保持静止。点 **① 初始化定位**，在地图上从机器人实际位置向机头朝向拖动，再点 **确认起点 / 初始化定位**。已有定位且显示“导航就绪”时可直接选终点。
2. 设置平移限速（页面默认 **0.15 m/s**）。点 **② 设置终点**，在目标位置按下并向期望的最终朝向拖动，再点 **确认终点 / 开始导航**。
3. 需要调整停车位置时，等当前任务结束，在微调区输入步长或转角，点前进、后退、左移、右移或原地转向按钮。
4. **取消当前任务**用页面的 **停止 / 取消导航** 或 `Esc`，等待页面确认已静止。终端 2 按 `Ctrl+C` 会关闭网页连接，已提交的任务继续执行。**结束整个导航会话**在终端 1 按 `Ctrl+C`。

普通导航按路径弧长判断朝向。全程超过 10 米时，在 20% 到 80% 之间优先让车头朝向前方路线。起步、末段、短路线和手动微调均无路线朝向要求。到点仍按保存的目标朝向停车。

普通地图目标允许横移绕障。跟踪或规划失败后，导航从当前位置重新规划到原目标。普通速度下，累计 60 秒没有有效位移或到点朝向进展，任务会失败并请求停车。很低的限速会延长观察窗口；停车确认后可再次提交目标。等待期间也可点击 **停止 / 取消导航**。

## 直接通过 ROS 操作

保留终端 1。在 Mac 的另一个终端运行下面这行，进入已加载导航环境的 S1B Bash。以下 ROS 命令都在这个终端执行。

```bash
ssh -t S1B 'exec bash --rcfile /home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/navigation_env.sh -i'
```

**准备命令并查看当前状态。** 整段复制一次。`nav_cmd` 用于提交请求，`nav_state` 随时可以重新查看位置、定位质量和任务结果。

```bash
nav_owner=$(cat /proc/sys/kernel/random/uuid)
nav_cmd() {
  ros2 service call /towergo/nav/command astribot_nav_interfaces/srv/NavigationCommand "$1"
}
nav_state() {
  ros2 topic echo /towergo/nav/task_state std_msgs/msg/String --qos-reliability best_effort --field data --csv --once |
    jq '{pose, localization: .localization.status, localization_trusted, ready, task, phase, result, error}'
}
nav_state
```

**初始化定位。** 复制后，按提示输入机器人在当前地图上的实际 `x y yaw`，用空格分隔。位置单位为米，朝向单位为弧度（90° 约为 1.5708）。服务受理后，运行 `nav_state`，等到 `localization` 为 `succeeded`、`localization_trusted` 和 `ready` 均为 `true` 再提交目标。

```bash
read -rp '当前地图位姿 x(m) y(m) yaw(rad)：' x y yaw && nav_cmd "{operation: localize, owner: '$nav_owner', x: $x, y: $y, yaw: $yaw}"
```

**导航到地图目标。** 复制后输入目标位置、最终朝向和平移限速。用响应中的 `task_id` 对应 `nav_state` 的 `task`。返回 `accepted: true` 表示请求已受理；该任务显示 `phase: idle` 且 `result: succeeded` 时，已到点并确认停稳。拒绝原因在响应的 `reason` 中。

```bash
read -rp '目标 x(m) y(m) yaw(rad) speed(m/s)：' x y yaw speed && nav_cmd "{operation: goal, owner: '$nav_owner', x: $x, y: $y, yaw: $yaw, speed: $speed}"
```

**底盘微调。** 以下是两个独立的运动示例。每次执行需要的那一条，等任务完成后再发下一条。坐标跟随机器人，`x` 正向为前，`y` 正向为左，`yaw` 正向为左转。

向左平移 10 cm：

```bash
nav_cmd "{operation: nudge, owner: '$nav_owner', x: 0.0, y: 0.10, yaw: 0.0, speed: 0.05}"
```

原地左转 15°：

```bash
nav_cmd "{operation: nudge, owner: '$nav_owner', x: 0.0, y: 0.0, yaw: 0.2618, speed: 0.05}"
```

**取消当前任务。** 返回后用 `nav_state` 查看停车进度和最终结果。

```bash
nav_cmd '{operation: stop}'
```

## 观察模式与键盘遥控

**只观察雷达与定位。** 在 Mac 的终端 1 选择观察模式，网页仍使用终端 2 的命令。观察模式不加载 SDK。

```bash
ssh -t S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation.sh --mode observe'
```

**只检查配置和依赖。** 这条命令执行完即退出，不启动导航节点。

```bash
ssh S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation.sh --mode hardware --check'
```

**切换到键盘遥控。** 先在终端 1 结束真机导航，让执行器交还底盘控制权，再从 Mac 启动遥控。此例使用线速度 0.03 m/s、角速度 0.075 rad/s。

```bash
ssh -t S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-teleop.sh --linear-speed 0.03 --angular-speed 0.075'
```

只演练键盘操作可用：

```bash
ssh -t S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-teleop.sh --dry-run'
```
