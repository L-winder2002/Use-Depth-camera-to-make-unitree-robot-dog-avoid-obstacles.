import colorsys
import numpy as np
import os
import torch
import torch.nn as nn
from PIL import ImageDraw, ImageFont
from .nets.yolo import YoloBody
from .utils.utils import (cvtColor, get_classes, preprocess_input,
                         resize_image, show_config)
from .utils.utils_bbox import DecodeBox
from ament_index_python.packages import get_package_share_directory

class YOLO(object):
    _defaults = {
        "model_path"        : 'model_data/yolov8_m.pth',
        "classes_path"      : 'model_data/coco_classes.txt',
        #---------------------------------------------------------------------#
        #   输入图片的大小，必须为32的倍数。
        #---------------------------------------------------------------------#
        "input_shape"       : [640, 480],
        #------------------------------------------------------#
        #   所使用到的yolov8的版本：
        #   n : 对应yolov8_n
        #   s : 对应yolov8_s
        #   m : 对应yolov8_m
        #   l : 对应yolov8_l
        #   x : 对应yolov8_x
        #------------------------------------------------------#
        "phi"               : 'm',
        #---------------------------------------------------------------------#
        #   只有得分大于置信度的预测框会被保留下来
        #---------------------------------------------------------------------#
        "confidence"        : 0.5,
        #---------------------------------------------------------------------#
        #   非极大抑制所用到的nms_iou大小
        #---------------------------------------------------------------------#
        "nms_iou"           : 0.3,
        #---------------------------------------------------------------------#
        #   该变量用于控制是否使用letterbox_image对输入图像进行不失真的resize，
        #   在多次测试后，发现关闭letterbox_image直接resize的效果更好
        #---------------------------------------------------------------------#
        "letterbox_image"   : True,
        #-------------------------------#
        #   是否使用Cuda
        #   没有GPU可以设置成False
        #-------------------------------#
        "cuda"              : True,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    #---------------------------------------------------#
    #   初始化YOLO
    #---------------------------------------------------#
    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)
            self._defaults[name] = value 

        # 使用 ROS 2 的 ament API 获取包路径
        package_path = get_package_share_directory('vision_obstacle_detection')
        model_data_path = os.path.join(package_path, 'model_data')

        # 更新路径为绝对路径
        self.model_path = os.path.join(model_data_path, 'yolov8_m.pth')
        self.classes_path = os.path.join(model_data_path, 'coco_classes.txt')

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        if not os.path.exists(self.classes_path):
            raise FileNotFoundError(f"Classes file not found: {self.classes_path}")
   
        #---------------------------------------------------#
        #   获得种类和先验框的数量
        #---------------------------------------------------#
        self.class_names, self.num_classes  = get_classes(self.classes_path)
        self.bbox_util                      = DecodeBox(self.num_classes, (self.input_shape[0], self.input_shape[1]))

        #---------------------------------------------------#
        #   画框设置不同的颜色
        #---------------------------------------------------#
        hsv_tuples = [(x / self.num_classes, 1., 1.) for x in range(self.num_classes)]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        self.generate()

        show_config(**self._defaults)

    #---------------------------------------------------#
    #   生成模型
    #---------------------------------------------------#
    def generate(self, onnx=False):
        #---------------------------------------------------#
        #   建立yolo模型，载入yolo模型的权重
        #---------------------------------------------------#
        self.net    = YoloBody(self.input_shape, self.num_classes, self.phi)
        device      = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.load_state_dict(torch.load(self.model_path, map_location=device))
        self.net    = self.net.fuse().eval()
        print('{} model, and classes loaded.'.format(self.model_path))
        if not onnx:
            if self.cuda:
                self.net = nn.DataParallel(self.net)
                self.net = self.net.cuda()

    #---------------------------------------------------#
    #   获取距离
    #---------------------------------------------------#
    def get_distance(self,top, left, bottom, right,depth_image):  
        # 2. 确保边界框在图像范围内
        height, width = depth_image.shape
        top = max(0, min(top, height-1))
        left = max(0, min(left, width-1))
        bottom = max(0, min(bottom, height-1))
        right = max(0, min(right, width-1))
        # 裁剪出深度图中的感兴趣区域（ROI）
        depth_roi = depth_image[top:bottom, left:right]  # 对 NumPy 数组进行切片操作
        
        # 现在可以处理 depth_roi 或计算最小深度
        min_distance = np.min(depth_roi[depth_roi > 0])  # 排除无效的深度值（如 0）
        if min_distance.size == 0:
        # 没有有效深度数据时的处理
            print("警告: 监测区域内没有有效深度值")
            return 0.1  # 或者返回None或其他标志值
        return min_distance/1000
             
    #---------------------------------------------------#
    #   检测图片
    #---------------------------------------------------#
    def detect_image(self, image,depth_image):
        coordinate_box = []
        #---------------------------------------------------#
        #   计算输入图片的高和宽
        #---------------------------------------------------#
        image_shape = np.array(np.shape(image)[0:2])
        #---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        #---------------------------------------------------------#
        image       = cvtColor(image)
        #---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        #---------------------------------------------------------#
        image_data  = resize_image(image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        #---------------------------------------------------------#
        #   添加上batch_size维度
        #   h, w, 3 => 3, h, w => 1, 3, h, w
        #---------------------------------------------------------#
        image_data  = np.expand_dims(np.transpose(preprocess_input(np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            #---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            #---------------------------------------------------------#
            outputs = self.net(images)
            outputs = self.bbox_util.decode_box(outputs)
            #---------------------------------------------------------#
            #   将预测框进行堆叠，然后进行非极大抑制
            #---------------------------------------------------------#
            results = self.bbox_util.non_max_suppression(outputs, self.num_classes, self.input_shape, 
                        image_shape, self.letterbox_image, conf_thres = self.confidence, nms_thres = self.nms_iou)

            thickness   = 2
            # 绘制固定绿色框代表机器狗可通行范围
            fixed_top = int(image.size[1] * 0.4)  # 距离顶部40%的位置
            fixed_bottom = fixed_top + 300  # 框的高度100像素
            fixed_left = int(image.size[0] * 0.3)  # 距离左侧40%的位置
            fixed_right = int(image.size[0] * 0.7)  # 左侧60%位置
            m=[]
            m.append(round(fixed_left/image.size[0],2))
            m.append(round(fixed_right/image.size[0],2))
            green_color = (0, 255, 0)  # 绿色框
            red_color = (255, 0, 0)  # 红色框---侵入到机器狗同行范围的障碍物框

            draw_fixed = ImageDraw.Draw(image)
            t=self.get_distance(fixed_top,fixed_left,fixed_bottom,fixed_right,depth_image)#get distance
            m.append(t)
            #如果可通行范围内有小于1.5m的东西，则在面前直接增加一个障碍物 保证不会由于yolo未检测出障碍物而发生碰撞
            if t<0.4:
                coordinate_box.append(m)
            #---------------------------------------------------------#
            #   设置字体
            #---------------------------------------------------------#
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
            except IOError:
                font = ImageFont.load_default()  # 使用默认字体

            for i in range(thickness):
                draw_fixed.rectangle(
                    [fixed_left + i, fixed_top + i, fixed_right - i, fixed_bottom - i],
                    outline=green_color
                )
            # 在框角绘制 t 的值
            label = '{} {:.2f}m'.format("Distance", t)  # 距离文本，保留两位小数
            # 使用 textbbox 计算文本边界框（替代 textsize）
            bbox = draw_fixed.textbbox((0, 0), label, font=font)
            label_width = bbox[2] - bbox[0]  # 文本宽度
            label_height = bbox[3] - bbox[1]  # 文本高度

            # 选择文本位置，例如左上角
            text_origin = (fixed_left, fixed_top - label_height - 5)  # 向上偏移 5 像素
            # 绘制文本背景框
            draw_fixed.rectangle([text_origin, (text_origin[0] + label_width, text_origin[1] + label_height)], fill=green_color)
            # 绘制文本
            draw_fixed.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw_fixed  
        
            if results[0] is None: 
                return image,coordinate_box
            

            top_label   = np.array(results[0][:, 5], dtype = 'int32')
            top_conf    = results[0][:, 4]
            top_boxes   = results[0][:, :4]
        #---------------------------------------------------------#
        #   图像绘制
        #---------------------------------------------------------#te)  
        
        #绘制检测框
        for i, c in list(enumerate(top_label)):
            coordinate=[]
            predicted_class = self.class_names[int(c)]
            box             = top_boxes[i]
            score           = top_conf[i]

            top, left, bottom, right = box

            top     = max(0, np.floor(top).astype('int32'))
            left    = max(0, np.floor(left).astype('int32'))
            bottom  = min(image.size[1], np.floor(bottom).astype('int32'))
            right   = min(image.size[0], np.floor(right).astype('int32'))

            depth_distance=self.get_distance(top, left, bottom, right,depth_image)
            coordinate.append(round(left/image.size[0],2))
            coordinate.append(round(right/image.size[0],2))
            coordinate.append(depth_distance)
            coordinate_box.append(coordinate)

            label = '{} {:.2f}m'.format(predicted_class, depth_distance)
            draw = ImageDraw.Draw(image)
            bbox = draw.textbbox((0, 0), label, font=font)
            label_width = bbox[2] - bbox[0]  # 文本宽度
            label_height = bbox[3] - bbox[1]  # 文本高度
            text_origin = (left, top - label_height - 5)  # 向上偏移 5 像素
            
            for i in range(thickness):
                draw.rectangle([left + i, top + i, right - i, bottom - i], outline=red_color)
            # 绘制文本背景框
            draw.rectangle([text_origin, (text_origin[0] + label_width, text_origin[1] + label_height)], fill=red_color)
            # 绘制文本
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw
        return image,coordinate_box