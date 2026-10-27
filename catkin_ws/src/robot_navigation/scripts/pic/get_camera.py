#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.cv_image = None
        self.image_count = 1
        self.window_name = "Press Q to save, ESC to exit"
        self.path="/home/GGB/catkin_ws/src/robot_navigation/scripts/pic/"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                # 显示图像
                cv2.imshow(self.window_name, self.cv_image)
                
                # 检测键盘输入
                key = cv2.waitKey(1) & 0xFF
                
                if key == 27:  # ESC键退出
                    rospy.signal_shutdown("User pressed ESC")
                    break
                elif key == ord('q') or key == ord('Q'):
                    # 保存图像
                    filename = self.path+f"{self.image_count}.jpg"
                    cv2.imwrite(filename, self.cv_image)
                    rospy.loginfo(f"Saved {filename}")
                    self.image_count += 1
            
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_saver_node')
    saver = ImageSaver()
    saver.run()