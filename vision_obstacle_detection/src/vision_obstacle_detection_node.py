import PIL.Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge  # 用于将ROS图像消息转换为OpenCV格式图像
import cv2
import numpy as np
import json
import math
from .a_star_m import planner
from .yolo_box import YOLO
from sensor_msgs.msg import Image as SensorImage  # 使用别名避免与PIL.Image冲突
from unitree_api.msg import Request
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import make_interp_spline


class VisionObstacleDetectionNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.gx,self.gy=5,0  #输入终点坐标 gx正值代表机器狗正前方，gy正值代表机器狗正左方
        self.sign=0
        self.obstacle_x,self.obstacle_y=[],[]
        self.yolo=YOLO()
        self.position = None # 用来存储机器人位置   
        # 创建发布者，用于发布控制指令
        self.control_publisher = self.create_publisher(Request, '/api/sport/request', 10)

        # 创建订阅深度相机图像话题的订阅者，需替换为实际话题名称
        self.depth_camera_subscription = self.create_subscription(
            SensorImage,
            'camera/camera/aligned_depth_to_color/image_raw',  # 替换为实际深度相机图像话题名称
            self.depth_camera_callback,
            10)
        self.depth_camera_subscription  # 防止未使用变量警告
        
        # 创建订阅普通相机图像话题的订阅者
        self.camera_subscription = self.create_subscription(
            SensorImage,
            'camera/camera/color/image_raw',
            self.camera_callback,
            10)
        self.camera_subscription  # 防止未使用变量警告

        self.bridge = CvBridge()  # 创建cv_bridge实例，用于图像格式转换

        self.odom_subscription = self.create_subscription(   
            PoseStamped,
            '/utlidar/robot_pose',
            self.odom_callback,
            10
        )
        self.odom_subscription  # 防止未使用变量警告

        # 创建定时器，每秒调用一次 timer_callback
        self.timer = self.create_timer(0.2, self.timer_callback)

    def odom_callback(self, msg):
        """
        处理 /unitree_go2/odom 话题消息的回调函数，这里简单打印消息内容示例
        你可以根据实际需求进一步解析和处理具体的字段信息，比如位置、姿态等
        """
        self.position = (msg.pose.position.x, msg.pose.position.y)
        self.orientation=(msg.pose.orientation.w,msg.pose.orientation.x
                          ,msg.pose.orientation.y,msg.pose.orientation.z)


    def depth_camera_callback(self, msg):
        try:
            # 将深度图像消息转换为 OpenCV 图像
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")  # 假设深度图像是32位浮动图像
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {str(e)}")
        
    def camera_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像,默认的BGR格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 将OpenCV图像（numpy.ndarray）转换为PIL.Image格式
            self.pil_image = PIL.Image.fromarray(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image or detecting objects: {str(e)}")


    def timer_callback(self):
        """
        定时器回调函数，每秒执行一次
        """
        pil_image=self.pil_image
        depth_image=self.depth_image
    
        # 进行目标检测
        detected_objects_image, coordinate = self.yolo.detect_image(pil_image, depth_image)
        
        position_x, position_y = self.position  #取出机器狗的当前位置信息
        
        orientation_w,orientation_x,orientation_y,orientation_z=self.orientation  #取出旋转信息 通过这个四元数组可以计算出朝向角yaw
        
        # 根据旋转信息的四元数组计算偏航角 (yaw) yaw=0时朝向x轴正方向
        siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
        cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.sign==0:
            ttx=position_x+self.gx*math.cos(current_yaw)-self.gy*math.sin(current_yaw)
            tty=position_y+self.gx*math.sin(current_yaw)+self.gy*math.cos(current_yaw)
            self.gx=ttx
            self.gy=tty
            self.get_logger().info(f"gx={self.gx},gy={self.gy},yaw={current_yaw}")
            self.sign=1
        self.get_logger().info(f"机器狗当前位置={position_x},{position_y},终点位置={self.gx},{self.gy}")  #打印机器狗当前位置
        path = planner()
        path_x, path_y ,obstacle_x,obstacle_y= path.path_planner(position_x, position_y, coordinate,current_yaw,self.gx,self.gy,self.obstacle_x,self.obstacle_y)  # 传入机器狗位置和检测框坐标，终点坐标
        path_x.reverse()
        path_y.reverse()
        path_x[0]=position_x
        path_y[0]=position_y
        self.get_logger().info(f"path_x={path_x},path_y={path_y}")
        if len(path_x) == 1:  #防止终点被占用规划不出路径报错
            path_x.extend([path_x[0]] * 2) 
            path_y.extend([path_y[0]] * 2)

        if math.dist((self.gx,self.gy),(position_x, position_y))>0.2:
            dx=path_x[1]-position_x
            dy=path_y[1]-position_y
            target_yaw = math.atan2(path_y[1] - position_y, path_x[1] - position_x)

            vx = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
            vy = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)
            vyaw=target_yaw-current_yaw

            if vyaw>0.3 or vyaw<-0.3:
                vx=0
                vy=0
                vyaw=max(-0.8,min(vyaw,0.8))
                move_cmd = {
                    "x": float(vx),
                    "y": float(vy), 
                    "z": float(vyaw)  # 注意：官方使用"z"表示yaw速度
                }
            else:
                vx=max(-0.5,min(vx,0.5))  #限制速度不太大不太小
                if vx > 0:
                    vx = max(0.2, vx)  # 正向时不低于 0.2
                elif vx < 0:
                    vx = min(-0.2, vx)  # 负向时不高于 -0.2
                vy=max(-0.1,min(vy,0.1))
                vyaw=max(-0.8,min(vyaw,0.8))
                move_cmd = {
                    "x": float(vx),
                    "y": float(vy), 
                    "z": float(vyaw)  # 注意：官方使用"z"表示yaw速度
                }
            # 创建 Request 消息
            req = Request()
            # 设置 API ID
            req.header.identity.api_id = 1008  # ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW
            req.parameter = json.dumps(move_cmd, separators=(',', ':'))
            # 发布 Request 消息
            self.control_publisher.publish(req)
        else:
            self.get_logger().info(f"机器狗已经到达终点")
        

        '''
        可视化部分
        '''
        # 将PIL图像转换回OpenCV格式
        detected_objects_image = np.array(detected_objects_image)
        # 在OpenCV窗口中显示目标检测后的图像
        cv2.imshow("Camera Feed", detected_objects_image)
        # 等待 50ms 并刷新窗口
        cv2.waitKey(50)

      
        image_size = 800  # 增大画布尺寸
        padding = 100     # 增加边距，确保所有点都能显示

        # 创建白色背景
        image = np.ones((image_size-100, image_size, 3), dtype=np.uint8) * 255

        # 自动计算数据范围，确保所有点都能显示
        min_x = min(min(obstacle_x), position_x, self.gx, min(path_x))
        max_x = max(max(obstacle_x), position_x, self.gx, max(path_x))
        min_y = min(min(obstacle_y), position_y, self.gy, min(path_y))
        max_y = max(max(obstacle_y), position_y, self.gy, max(path_y))

        # 计算缩放比例，保留10%的边距
        data_range_x = max_x - min_x
        data_range_y = max_y - min_y
        data_range = max(data_range_x, data_range_y) * 1.2  # 增加20%的边距

        # 计算缩放比例和偏移量
        usable_size = image_size - 2 * padding
        scale = usable_size / data_range
        offset_x = padding - min_x * scale
        offset_y = padding + max_y * scale  # 因为y轴是向下的

        # 坐标转换函数
        def transform(x, y):
            px = int(x * scale + offset_x)
            py = int(offset_y - y * scale)  # y轴翻转
            return px, py

        # 画障碍物（增大点的大小和对比度）
        for x, y in zip(obstacle_x, obstacle_y):
            px, py = transform(x, y)
            cv2.circle(image, (px, py), 5, (50, 50, 50), -1)  # 深灰色，更明显

        # 画路径（红色线，加粗）
        points = [transform(x, y) for x, y in zip(path_x, path_y)]
        for i in range(len(points) - 1):
            cv2.line(image, points[i], points[i+1], (0, 0, 255), 3)  # 加粗红线

        # 画目标点（蓝色，增大尺寸）
        px, py = transform(self.gx, self.gy)
        cv2.circle(image, (px, py), 8, (180, 0, 0), -1)  # 更深的蓝色
        cv2.circle(image, (px, py), 8, (0, 0, 0), 1)     # 黑色边框

        # 画起点（绿色，增大尺寸）
        px, py = transform(position_x, position_y)
        cv2.circle(image, (px, py), 8, (0, 255, 0), -1)  # 更深的绿色
        cv2.circle(image, (px, py), 8, (0, 0, 0), 1)    # 黑色边框
        # 添加朝向箭头（绿色箭头）
        arrow_length = 25  # 箭头长度
        end_x = px + arrow_length * np.cos(current_yaw)  # 计算箭头终点x
        end_y = py - arrow_length * np.sin(current_yaw)  # y轴向下，所以用减号
        cv2.arrowedLine(
            image, 
            (px, py), 
            (int(end_x), int(end_y)), 
            (0, 255, 0),  # 绿色箭头
            2,  # 线宽
            tipLength=0.3  # 箭头头部长度比例
        )

        # 显示图像
        cv2.namedWindow("Path Planning Visualization", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Path Planning Visualization", 1000, 1000)
        cv2.imshow("Path Planning Visualization", image)
        cv2.waitKey(1)


                
    
def main():
    rclpy.init()
    vision_node = VisionObstacleDetectionNode('vision_obstacle_detection_node')  # 明确传入节点名称参数
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()