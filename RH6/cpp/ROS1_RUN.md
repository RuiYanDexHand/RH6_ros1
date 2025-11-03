# RH6 ROS1 C++ 运行步骤与命令（Ubuntu）

适用于 Ubuntu + ROS1（推荐 Noetic/20.04，或 Melodic/18.04）。本指南覆盖 rh6_cmd、rh6_msg、rh6_ctrl 三个包的构建与运行。

## 1. 安装依赖

1. 安装 ROS1（Noetic 或 Melodic），并完成 rosdep 初始化。
2. 安装构建与系统库（若 rosdep 未自动安装）：

```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y libeigen3-dev libpinocchio-dev
```

提示：首次使用建议运行一次 rosdep（第 3 步会覆盖）：

```bash
sudo rosdep init  # 已初始化过可跳过
rosdep update
```

## 2. 创建 catkin 工作空间

```bash
# 选择一个工作目录
mkdir -p ~/rh6_ws/src
cd ~/rh6_ws
```

## 3. 将本仓库的 3 个包加入工作空间

假设你把本仓库放在 /path/to/repo。将 RH6/cpp 下的包软链接到工作空间：

```bash
cd ~/rh6_ws/src
ln -s /path/to/repo/RH6/cpp/rh6_msg .
ln -s /path/to/repo/RH6/cpp/rh6_cmd .
ln -s /path/to/repo/RH6/cpp/rh6_ctrl .
```

- 若你不方便使用软链接，也可以直接复制这三个包到 ~/rh6_ws/src。

## 4. 配置 ROS 环境并安装包依赖

```bash
# 使用你已安装的 ROS 发行版（以下以 Noetic 为例）
source /opt/ros/noetic/setup.bash

cd ~/rh6_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

如果提示找不到 Pinocchio/Eigen，请再次执行：

```bash
sudo apt install -y libeigen3-dev libpinocchio-dev
```

## 5. 编译

```bash
cd ~/rh6_ws
catkin_make
source devel/setup.bash
```

## 6. 运行前设置（Ryhand 动态库路径）

`rh6_ctrl` 依赖源代码目录下的私有库（`rh6_ctrl/lib/*.so`）。为确保运行时能找到这些库，请在每个新终端运行前设置：

```bash
# 将 Ryhand 库目录加入运行库搜索路径
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(readlink -f ~/rh6_ws/src/rh6_ctrl/lib)
```

或永久安装库（可选）：

```bash
sudo cp ~/rh6_ws/src/rh6_ctrl/lib/libRyhand*.so /usr/local/lib/
sudo ldconfig
```

## 7. 运行命令

开 3 个终端，分别执行：

1) 启动 roscore
```bash
source /opt/ros/noetic/setup.bash
roscore
```

2) 运行控制节点（rh6_ctrl）
```bash
source /opt/ros/noetic/setup.bash
source ~/rh6_ws/devel/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(readlink -f ~/rh6_ws/src/rh6_ctrl/lib)
rosrun rh6_ctrl rh6_ctrl_node
```

3) 运行测试或演示
- 测试节点：
```bash
source /opt/ros/noetic/setup.bash
source ~/rh6_ws/devel/setup.bash
rosrun rh6_ctrl rh6_test_node
```

- 运动学演示：
```bash
source /opt/ros/noetic/setup.bash
source ~/rh6_ws/devel/setup.bash
rosrun rh6_ctrl rh6_kinematics_node
```

- 使用启动文件（BothHands）：
```bash
source /opt/ros/noetic/setup.bash
source ~/rh6_ws/devel/setup.bash
roslaunch rh6_ctrl BothHands.py
```

- 启动脚本（scripts/dual_sine_cmd.py）发布双手正弦命令：
（建议先按上一步启动 BothHands.py，使左右手命名空间与脚本默认主题匹配）
```bash
source /opt/ros/noetic/setup.bash
source ~/rh6_ws/devel/setup.bash
rosrun rh6_ctrl dual_sine_cmd.py
```

- 可选：调整脚本私有参数（发布频率/频率/基值/幅度）
```bash
rosrun rh6_ctrl dual_sine_cmd.py _rate:=50.0 _freq:=0.25 _base:=2000 _amp:=800
```

## 8.（可选）配置 CAN 接口（SocketCAN）

若你的硬件通过 `can0` 通讯，可参考：

```bash
# 以 1Mbps 为例（根据硬件调整）
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

如果应用需要特权（如实时优先级或访问 CAN），可考虑：
- 以 `sudo -E` 保留环境变量启动节点（需同步设置 LD_LIBRARY_PATH）。
- 或将可执行加入适当的权限组/能力。

## 9. 常见问题

- Pinocchio/CMake 找不到：
  - 执行 `sudo apt install libpinocchio-dev libeigen3-dev`
  - 重新 `catkin_make`，并确认 `CMakeLists.txt` 中 pinocchio 目标已找到（本包已兼容 `pinocchio::pinocchio` 和 `pinocchio` 两种形式）。
- 运行时报 `libRyhand*.so: cannot open shared object file`：
  - 按第 6 步设置 `LD_LIBRARY_PATH`，或复制库到 `/usr/local/lib` 并 `sudo ldconfig`。
- 消息/服务未生成：
  - 确保 `rh6_cmd`、`rh6_msg` 在 `~/rh6_ws/src`，且先 `catkin_make` 成功，运行前已 `source devel/setup.bash`。

---

如需我把上述命令整理为一键脚本（setup.sh/ run.sh），告诉我你的 ROS 发行版（Noetic/Melodic）和你的仓库路径，我可以直接生成。