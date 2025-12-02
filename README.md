# Unitree Go2 SLAM Toolbox 项目  

本项目是一个基于 ROS2 的 Unitree Go2 机器狗开发框架，集成了 SLAM 建图、自主导航、感知处理等核心功能，提供了完整的机器人开发和测试环境。

## 📊 项目架构

该项目采用标准 ROS2 colcon 工作空间结构，包含以下核心功能包：

### 1. 基础层
- **go2_core**: 核心功能包，提供机器人基本控制接口，依赖 unitree_go、unitree_api 等底层库
- **go2_driver**: 驱动程序包，负责与 Go2 机器人硬件通信
- **go2_twist_bridge**: 速度命令转换桥接，处理不同坐标系间的速度控制转换

### 2. 感知层
- **go2_perception**: 感知处理包，包含点云累积和转换功能
  - cloud_accumulation: 点云累积处理
  - pointcloud_to_laserscan_node: 将 3D 点云转换为 2D 激光扫描数据

### 3. 建图层
- **go2_slam**: 基于 slam_toolbox 实现的 SLAM 功能包，配置为在线异步建图模式

### 4. 导航层
- **go2_navigation2**: 基于 Nav2 框架的导航功能包，配置路径跟随控制器和目标检查器

### 5. 描述层
- **go2_description**: 机器人模型描述包，包含 URDF 模型文件和 3D 网格文件

### 6. 通信测试
- **message_test**: Zenoh 通信测试目录，包含导航点发送和反馈接收功能

## 🛠️ 硬件准备  
1. Unitree Go2 PRO(破解)/EDU 版机器狗  
2. 安装 ROS2 Humble 的 PC 主机  
3. 5-10米网线（用于机器狗与 PC 连接）  

## ✅ 已实现功能  
- Rviz2 中机器狗模型可视化  
- 点云累积及 `PointCloud2_to_LaserScan` 消息转换  
- 支持 ROS2 官方键盘控制节点  
- 基于 IMU 融合里程计（odom）数据  
- 集成 slam-toolbox 实现建图功能  
- 基于 Nav2 实现自主导航功能  
- Zenoh 通信集成，支持远程导航点发送和反馈接收
- 多机器人场景支持（通过 robot_id 区分）

## 🔧 开发环境配置  
推荐参考以下教程完成网络环境搭建（**网络环境搭建十分重要，一定要完成**）：  
- 宇树官方ros2 SDK：
https://github.com/unitreerobotics/unitree_ros2
安装完之后需要在本项目的工作空间
```
source setup.sh
```

## 📦 依赖安装  
1. 安装机器人定位融合包  
   ```bash
   sudo apt update && sudo apt install ros-humble-robot-localization
   ```  

2. 安装建图工具  
   ```bash
   sudo apt update && sudo apt install ros-humble-slam-toolbox
   ```  

3. 安装导航框架  
   ```bash
   sudo apt update && sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

4. 安装 Zenoh Python 客户端（用于通信测试）
   ```bash
   pip install zenoh
   ```

5. 其他依赖：编译时若提示缺少包，可根据报错信息用 `sudo apt install ros-humble-<缺失包名>` 安装  

## 🚀 快速启动步骤  

### 基本功能启动
1. **创建并进入工作空间**  
   ```bash
   mkdir -p go2_ws_toolbox/src && cd go2_ws_toolbox/src
   ```  

2. **克隆仓库**  
   ```bash
   git clone https://github.com/tkdggg/slam_nav_with_go2_pro.git
   ```  

3. **编译工作空间**  
   ```bash
   cd .. && colcon build --parallel-workers 1
   ```  

4. **启动 SLAM 建图（包含可视化）**  
   ```bash
   source install/setup.bash
   ros2 launch go2_core go2_start.launch.py
   ```  

5. **启动键盘控制（控制机器狗移动建图）**  
   ```bash
   source install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

### 新开终端导航功能启动
```bash
source install/setup.bash
ros2 launch go2_navigation2 go2_nav2.launch.py
```

### Zenoh 通信测试
1. **启动导航接收端**
   ```bash
   cd message_test
   python exe.py
   ```

2. **发送导航点**
   ```bash
   cd message_test
   python test.py
   ```

## ⚙️ Zenoh 通信说明

项目集成了 Zenoh 分布式通信框架，支持通过以下主题进行通信：

- **动作主题**: `edge/{robot_id}/action` - 用于发送导航目标点
- **反馈主题**: `edge/feedback` - 用于接收导航执行结果

导航点数据格式示例：
```json
{
  "point_id": "point_1",
  "x": 1.0,
  "y": 0.5,
  "theta": 0.0
}
```

## 📝 注意事项

1. **建图建议**：
   - 移动速度控制在 0.3m/s 左右
   - 步态选择「经典模式」以保证稳定性
   - 确保环境中有足够的特征点以便 SLAM 算法工作

2. **导航参数调整**：
   - 导航参数可在 `go2_nav2.launch.py` 中调整
   - 可根据实际环境修改控制器参数以获得更好的导航效果

3. **多机器人配置**：
   - 在测试文件中修改 `ROBOT_ID` 常量以支持多机器人场景

## 🤝 贡献指南

欢迎提交 Issue 和 Pull Request 来改进这个项目。如果您有任何问题或建议，请随时在 GitHub 上提出。

## 许可证

[MIT License](LICENSE)