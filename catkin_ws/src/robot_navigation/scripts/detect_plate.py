#!/usr/bin/env python3

import rospy
import cv2
import base64
import requests
import threading
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_navigation.srv import *
from PIL import Image as PILImage, ImageDraw, ImageFont


API_KEY = "y5lDLay9bXIwUj8PMKtjHPXz"
SECRET_KEY = "E9ykq0gc10nuJvu4UKtDBLuvLDeSQB0L"
ACCESS_TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
PLATE_API_URL = "https://aip.baidubce.com/rest/2.0/ocr/v1/license_plate"

# 字体配置
FONT_PATH = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"  
FONT_SIZE = 32

class PlateRecognizer:
    def __init__(self):
        rospy.init_node('plate_recognition_service', anonymous=True)
        
        # 创建ROS服务
        self.service = rospy.Service('/recognize_plate', detect, self.handle_recognition)
        
        # 图像订阅与发布
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/recognized_image", Image, queue_size=1)
        self.save_path = '/home/TT/catkin_ws/src/robot_navigation/scripts/pic/plate.jpg'
        
        # 获取百度API访问令牌
        self.access_token = self.get_access_token()
        if not self.access_token:
            rospy.logerr("无法获取百度API访问令牌，请检查API_KEY和SECRET_KEY。")
            rospy.signal_shutdown("API认证失败")
            
        rospy.loginfo("车牌识别服务已启动！")

    # 图像接收回调
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as e:
            rospy.logerr(f"图像转换错误: {str(e)}")

    # 获取百度API Token
    def get_access_token(self):
        params = {
            "grant_type": "client_credentials",
            "client_id": API_KEY,
            "client_secret": SECRET_KEY
        }
        try:
            response = requests.post(ACCESS_TOKEN_URL, params=params)
            response.raise_for_status()
            return response.json().get("access_token")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Access Token请求失败: {str(e)}")
            return None

    # 调用百度API识别车牌
    def recognize_plate(self, cv_image):
        if cv_image is None:
            return None
        
        _, buffer = cv2.imencode('.jpg', cv_image)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        payload = {'image': img_base64, 'access_token': self.access_token}
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        
        try:
            response = requests.post(PLATE_API_URL, headers=headers, data=payload)
            response.raise_for_status()
            return response.json()
        except Exception as e:
            rospy.logerr(f"车牌识别API错误: {str(e)}")
            return None

    # 处理车牌号编码问题
    def decode_plate_number(self, raw_text):

        if not raw_text:
            return "N/A"

        # 尝试UTF-8转义解码
        try:
            decoded = raw_text.encode('latin1').decode('utf-8')
            if any('\u4e00' <= ch <= '\u9fff' for ch in decoded):
                return decoded
        except Exception:
            pass

        # 尝试unicode_escape解码
        try:
            decoded = raw_text.encode('utf-8').decode('unicode_escape')
            if any('\u4e00' <= ch <= '\u9fff' for ch in decoded):
                return decoded
        except Exception:
            pass

        # 如果都不成功，直接返回原始字符串
        return raw_text

    # 绘制识别结果
    def draw_plate_info(self, image, result_json):
        draw_image = image.copy()

        if 'words_result' not in result_json:
            return draw_image, "RECOGNITION_FAILED_NO_WORDS"

        words_result = result_json['words_result']
        plate_number_raw = words_result.get('number', 'N/A')
        plate_number = self.decode_plate_number(plate_number_raw)
        location = words_result.get('vertexes_location', None)

        if location and len(location) == 4:
            # 绘制车牌框
            points = np.array([[p['x'], p['y']] for p in location], np.int32)
            cv2.polylines(draw_image, [points], isClosed=True, color=(0, 255, 0), thickness=2)

            # 绘制中文文字
            x, y = location[0]['x'], location[0]['y']
            text_y = max(20, y - 10)

            pil_image = PILImage.fromarray(cv2.cvtColor(draw_image, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_image)

            try:
                font = ImageFont.truetype(FONT_PATH, FONT_SIZE)
            except IOError:
                rospy.logwarn(f"无法加载字体 {FONT_PATH}，使用默认字体。")
                font = ImageFont.load_default()

            draw.text((x, text_y), plate_number, font=font, fill=(255, 255, 0))
            draw_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

            rospy.loginfo(f"识别到车牌号: {plate_number}")
        else:
            rospy.logwarn("未检测到有效车牌位置。")

        return draw_image, plate_number

    # ROS服务处理逻辑
    def handle_recognition(self, req):
        if req.detect_flag == 3:
            rospy.loginfo("收到车牌识别请求，开始检测...")

            with self.image_lock:
                current_image = None if self.latest_image is None else self.latest_image.copy()

            if current_image is None:
                rospy.logwarn("当前没有可用图像！")
                return detectResponse("ERROR: No image available")

            result_json = self.recognize_plate(current_image)
            if result_json:
                result_image, plate_number = self.draw_plate_info(current_image, result_json)
                cv2.imwrite(self.save_path, result_image)
                rospy.loginfo(f"已保存识别结果图像: {self.save_path}")

                if plate_number and plate_number != 'N/A':
                    return detectResponse(plate_number)
                else:
                    return detectResponse("RECOGNITION_FAILED")
            else:
                rospy.logwarn("百度API未返回有效结果。")
                return detectResponse("RECOGNITION_FAILED")


# 主程序入口
if __name__ == '__main__':
    try:
        recognizer = PlateRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
