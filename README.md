# Use-Depth-camera-to-make-unitree-robot-dog-avoid-obstacles.
实时视觉避障与导航系统：**Intel RealSense D435i** 深度、**YOLOv8** 目标检测、**ROS 2** 感知-建图-规划链路、**A\*** 局部路径，适配 **Unitree Go2**（实机）与 **Isaac Sim**（仿真）。

> 论文：*Path planning and guidance for quadruple legged robots through obstacles using YOLOv8 and A-Star with ROS2 and Isaac Sim*（ICASSE，一作）  
> Demo：https://youtu.be/TyCdeRsappY

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue)
![Ultralytics YOLOv8](https://img.shields.io/badge/YOLOv8-ultralytics-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-green)

---

## ✨ Features
- **检测**：YOLOv8 实时识别障碍（人/椅子/货架等），支持自训权重  
- **深度融合**：2D 检测框与 D435i 对齐，取最小深度估距  
- **局部栅格/代价地图**：去噪、膨胀、可视化话题  
- **A\***：在局部地图上规划可行路径，输出 `Twist` 或 Unitree 接口指令  
- **仿真/实机双通道**：Isaac Sim 与 Unitree Go2 实机验证  
- **模块化**：感知、建图、规划、控制解耦，易于替换（DWA/TEB/Hybrid A*）

---

## 🗂️ Repo 结构
```
.
├─ model_data/                  # YOLO 权重/类别
├─ nets/                        # YOLO 推理/后处理
├─ unitree_api/                 # Unitree 自定义接口
├─ unitree_go/                  # 自定义 ROS2 msgs: motorstate / sportmodestate（如有）
├─ utils/                       # 几何/可视化/相机工具
├─ a_star_m.py                  # A* 实现
├─ yolo_box.py                  # YOLO 检测与 bbox 处理
├─ vision_obstacle_detection_node.py  # 主节点（感知→深度→地图→规划→控制）
└─ launch/                      # TODO: launch 文件
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
# 若用到：pip install rclpy transforms3d pandas
```
---

## ⚙️ 快速开始
1) 构建
```
mkdir -p ~/ws_unitree_depth/src && cd ~/ws_unitree_depth/src
git clone https://github.com/L-winder2002/Use-Depth-camera-to-make-unitree-robot-dog-avoid-obstacles.git
cd ..
colcon build
source install/setup.bash
```

2) 模型与配置
将 YOLO 权重与类别文件放入：
```
model_data/
├─ yolov8_*.pth         # TODO: 你的权重文件名
└─ coco_classes.txt
```

3) 启动相机（实机）
```
ros2 launch realsense2_camera rs_launch.py \
  align_depth:=true depth_width:=640 depth_height:=480
```
4) 运行避障节点
```
ros2 run vision_obstacle_detection vision_obstacle_detection_node
```
