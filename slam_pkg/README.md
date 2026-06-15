# slam_pkg

`slam_pkg` 是一个独立的 ROS 2 SLAM 仿真学习包。它参考了 `wpr_simulation2` 的 `robocup_home` 场景，但已经把仿真世界、机器人模型、家具模型、mesh 资源、SLAM 配置、RViz 配置和键盘控制节点移植到本包内部。

## 1. 包内容

主要目录如下：

```text
slam_pkg/
  config/
    slam_toolbox.yaml          # slam_toolbox 建图参数
    wpb_home_controller.yaml   # 原仿真控制配置资源
  launch/
    slam.launch.py             # 一键启动仿真 + SLAM + RViz
    robocup_home.launch.py     # 单独启动 Gazebo 仿真环境
    spawn_wpb_lidar.launch.py  # 生成带激光雷达的机器人
    spawn_objects.launch.py    # 生成家具和物体
  models/                      # 机器人和家具模型
  meshes/                      # 模型渲染用 mesh 和贴图
  rviz/
    slam.rviz                  # SLAM 可视化配置
  src/
    keyboard_vel_cmd.cpp       # 键盘速度控制节点
  worlds/
    robocup_home.world         # 室内墙体世界
```

## 2. 构建

进入 `unitree_ws` 工作区：

```bash
cd ~/ros/unitree_ws
colcon build --packages-select slam_pkg
source install/setup.bash
```

如果改过 launch、model、mesh、world 或 C++ 节点，建议重新构建并重新 source：

```bash
colcon build --packages-select slam_pkg
source install/setup.bash
```

## 3. 单独启动仿真环境

只打开 Gazebo 仿真，不启动 SLAM：

```bash
ros2 launch slam_pkg robocup_home.launch.py
```

该 launch 会执行以下流程：

1. 读取本包的 `worlds/robocup_home.world`。
2. 启动 `gazebo_ros` 的 `gzserver`。
3. 启动 `gazebo_ros` 的 `gzclient`。
4. 调用 `spawn_wpb_lidar.launch.py` 生成机器人。
5. 调用 `spawn_objects.launch.py` 生成床、沙发、桌子、柜子、椅子、瓶子等家具物体。

机器人默认出生位置在：

```text
x = -6.0
y = -0.5
yaw = 0.0
```

也可以在启动时修改：

```bash
ros2 launch slam_pkg robocup_home.launch.py \
  robot_pose_x:=-6.0 \
  robot_pose_y:=-0.5 \
  robot_pose_theta:=0.0
```

## 4. 启动 SLAM

启动仿真、`slam_toolbox` 和 RViz：

```bash
ros2 launch slam_pkg slam.launch.py
```

该 launch 默认调用本包自己的仿真环境：

```text
simulation_package = slam_pkg
simulation_launch_file = robocup_home.launch.py
```

SLAM 参数在：

```text
config/slam_toolbox.yaml
```

关键配置：

```yaml
use_sim_time: true
map_frame: map
odom_frame: odom
base_frame: base_footprint
scan_topic: /scan
mode: mapping
```

RViz 配置在：

```text
rviz/slam.rviz
```

RViz 中主要查看：

```text
/map
/scan
/robot_description
TF: map -> odom -> base_footprint
```

## 5. 键盘控制机器人

键盘控制节点已经从 `wpr_simulation2/src/keyboard_vel_cmd.cpp` 移植到：

```text
src/keyboard_vel_cmd.cpp
```

它发布速度到：

```text
/cmd_vel
```

机器人模型 `models/wpb_home_lidar.model` 中的 Gazebo 控制插件订阅 `cmd_vel`，因此可以直接控制仿真机器人。

推荐开两个终端。

终端 1：启动仿真或完整 SLAM：

```bash
cd ~/ros/unitree_ws
source install/setup.bash
ros2 launch slam_pkg slam.launch.py
```

终端 2：启动键盘控制：

```bash
cd ~/ros/unitree_ws
source install/setup.bash
ros2 run slam_pkg keyboard_vel_cmd
```

按键说明：

```text
w      前进加速
s      后退加速
a      向左平移加速
d      向右平移加速
q      左旋加速
e      右旋加速
space  刹车
x      退出
```

速度每次按键增加：

```text
linear_vel  = 0.1
angular_vel = 0.1
最大倍率    = 3
```

也就是最大速度约为：

```text
linear.x  = +/-0.3
linear.y  = +/-0.3
angular.z = +/-0.3
```

## 6. 家具渲染修复方式

Gazebo 中墙面能显示、家具不显示，通常是因为家具模型里的 mesh 路径无法被 Gazebo 正确解析。

原模型里使用的是：

```xml
<mesh filename="package://slam_pkg/meshes/bookshelft.dae" />
```

这种 `package://` 路径在某些 `spawn_entity.py -file` 场景下可能解析失败。参考 `sim_pkg` 中 bookshelf 的成功写法，本包现在在 launch 运行时临时生成 patched model，把 mesh 路径改成绝对 `file://` 路径。

例如运行前模型中是：

```xml
package://slam_pkg/meshes/bookshelft.dae
```

运行时会临时改成类似：

```text
file:///home/mscape/ros2/unitree_ws/install/slam_pkg/share/slam_pkg/meshes/bookshelft.dae
```

相关实现位置：

```text
launch/spawn_objects.launch.py
launch/spawn_wpb_lidar.launch.py
```

临时 patched model 会写到系统临时目录：

```text
/tmp/slam_pkg_*.model
```

源码中的 `models/*.model` 仍然保留 `package://slam_pkg/...`，便于维护；真正给 Gazebo spawn 的是 patched 后的临时文件。

## 7. 常见问题

### Duplicate package names not supported

如果出现：

```text
Duplicate package names not supported:
- slam_pkg:
  - src/slam_pkg
  - src/slam_pkg.original_nobody
```

说明 `unitree_ws/src` 下还有另一个带 `package.xml` 的备份目录。`colcon` 会扫描 `src` 下所有包，因此需要把备份目录移出 `src`，或者在备份目录中放置 `COLCON_IGNORE`。

当前应只保留：

```text
unitree_ws/src/slam_pkg/package.xml
```

可以检查：

```bash
find ~/ros/unitree_ws/src -maxdepth 3 -name package.xml -print
```

### 家具还是不显示

先确认重新构建并 source：

```bash
cd ~/ros/unitree_ws
colcon build --packages-select slam_pkg
source install/setup.bash
```

再确认安装目录里有资源：

```bash
ls install/slam_pkg/share/slam_pkg/models
ls install/slam_pkg/share/slam_pkg/meshes
```

如果 Gazebo 已经打开，建议关闭后重新启动 launch。

### 键盘控制没有反应

检查 `/cmd_vel` 是否有消息：

```bash
ros2 topic echo /cmd_vel
```

再启动键盘节点并按 `w`、`s`、空格等按键：

```bash
ros2 run slam_pkg keyboard_vel_cmd
```

键盘节点需要当前终端焦点，按键要在运行该节点的终端里输入。
