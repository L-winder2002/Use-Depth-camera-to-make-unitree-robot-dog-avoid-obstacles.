# Use-Depth-camera-to-make-unitree-robot-dog-avoid-obstacles.
实时视觉避障与导航系统：**Intel RealSense D435i** 深度、**YOLOv8** 目标检测、**ROS 2** 感知-建图-规划链路、**A\*** 局部路径，适配 **Unitree Go2**（实机）与 **Isaac Sim**（仿真）。

> 论文：*Path planning and guidance for quadruple legged robots through obstacles using YOLOv8 and A-Star with ROS2 and Isaac Sim*（ICASSE，一作）  
> Demo：https://youtu.be/TyCdeRsappY

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python 3.10](https://img.shields.io/badge/Python-3.10%2B-blue)
![Ultralytics YOLOv8](https://img.shields.io/badge/YOLOv8-ultralytics-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-green)

---

## ✨ Features
- **检测**：YOLOv8 实时识别障碍（人/椅子/货架等），支持自训权重  
- **深度融合**：2D 检测框与 D435i 对齐，取最小深度估距  
- **局部栅格/代价地图**：去噪、膨胀、可视化话题  
- **A\***：在局部地图上规划可行路径，输出Unitree 接口指令  
- **仿真/实机双通道**：Isaac Sim 与 Unitree Go2 实机验证  
- **模块化**：感知、建图、规划、控制解耦，易于替换（DWA/TEB/Hybrid A*）

---

## 🗂️ src文件结构
```
.
//（需要手动下载放入）├─ model_data/  # YOLO 权重/类别
├─ nets/                        # YOLO 推理/后处理
├─ unitree_api/                 # Unitree 自定义接口
├─ unitree_go/                  # 自定义 ROS2 msgs:
├─ utils/                       # 几何/可视化/相机工具
├─ a_star_m.py                  # A* 实现
├─ yolo_box.py                  # YOLO 检测与 bbox 处理
└─ vision_obstacle_detection_node.py  # 主节点（感知→深度→地图→规划→控制）
```
---

## 🛠️ 环境要求
- Ubuntu 22.04, ROS 2 Humble, Python 3.10
- Intel RealSense SDK & ROS 包：`realsense2_camera`
- （可选）CUDA/GPU 用于 YOLO 加速

```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera
pip install ultralytics opencv-python numpy pyrealsense2==2.* scipy matplotlib
```
---

## ⚙️ 快速开始
## 一、在ubuntu22.04中构建unitree go2环境
1) ctrl+alt+T 打开终端，克隆仓库。
```
git clone https://github.com/unitreerobotics/unitree_ros2
```
其中
- **cyclonedds_ws** 文件夹为编译和安装 Go2 机器人 ROS2 msg 的工作空间，在子文件夹 cyclonedds_ws/unitree/unitree_go和cyclonedds_ws/unitree/unitree_api中定义了Go2状态获取和控制相关的 ROS2 msg。
- **Go2_ROS2_example** 文件夹为 Go2 机器人 ROS2 下的相关例程。

2) 安装 Unitree ROS2 功能包
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

3) 编译 cyclone-dds

编译 cyclonedds 前请确保在启动终端时没有 source ros2 相关的环境变量，否则会导致 cyclonedds 编译报错。如果安装 ROS2 时在~/.bashrc中添加了 " source /opt/ros/humble/setup.bash "，需要修改 ~/.bashrc 文件将其删除.

在终端中执行以下操作编译 cyclone-dds:
```
cd ~/unitree_ros2/cyclonedds_ws/src
#克隆cyclonedds仓库
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds #编译cyclonedds
```
4) 编译 unitree_go 和 unitree_api 功能包
```
source /opt/ros/humble/setup.bash #source ROS2 环境变量
colcon build #编译工作空间下的所有功能包
```

## 二、将避障算法包合并到unitree go2环境中
1) 将下载好的vision_obstacle_detection放入unitree_ros2/中
2) 模型与配置
将 YOLO 权重与类别文件放入：

本项目实验权重文件：

model_data
链接: https://pan.baidu.com/s/13WsUAfC6qz3JykU1QeZ4OQ?pwd=n335 提取码: n335 

可直接下载后放入unitree_ros2/vision_obstacle_detection/src/

```
model_data/
├─ yolov8_m.pth        
└─ coco_classes.txt
```

3) 编译全部文件
```
cd ~/unitree_ros2
colcon build
```

## 三、连接宇树机器狗（参考网址：https://support.unitree.com/home/zh/developer/ROS2_service）
1) 使用网线连接 Go2 和计算机，使用 ifconfig 查看网络信息，确认机器人连接到的以太网网卡。
2) 接着打开网络设置，找到机器人所连接的网卡，进入 IPv4 ，将 IPv4 方式改为手动，地址设置为192.168.123.99，子网掩码设置为255.255.255.0，完成后点击应用，等待网络重新连接。
3) 打开 unitree_go2/setup.sh 文件,内容修改如下：(name="enp3s0" 根据上一步机器狗实际网卡名称填写)
```
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```
4) 完成上述配置后，建议重启一下电脑再进行测试。
确保机器人连接正确，打开终端输入 ros2 topic list,检查是否连接成功。

## 四、运行项目
1) 启动相机（实机）
```
ros2 launch realsense2_camera rs_launch.py （根据你的相机启动方式自行启动intel realsense2，然后根据相机输出的节点调整代码中的订阅节点）
```
2) 检验相机话题是否正常发布
```
ros2 topic list
```
正常情况下可以看到/camera/color/image_raw，/camera/depth/image_rect_raw等话题

3) source相关setup文件
```
cd unitree_go2
colcon build
source setup.sh
source install/setup.bash
```

4) 运行避障节点（保证机器狗连接成功，相机连接成功）
```
ros2 run vision_obstacle_detection vision_obstacle_detection_node
```
