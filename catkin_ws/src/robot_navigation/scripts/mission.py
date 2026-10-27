#!/usr/bin/env python3

import rospy
import actionlib
import traceback

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import pi
from std_srvs.srv import Empty
from robot_navigation.srv import *


class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('misson', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # 发布器（用于 shutdown 停车）
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 预先构造四元数（与原脚本的角度一一对应）
        euler_angles = (0, 0, pi/2, pi/2, -pi/2, pi/2, pi/2, pi, pi, -pi/2, pi)
        self.quaternions = [Quaternion(*quaternion_from_euler(0, 0, a)) for a in euler_angles]

        # 航点（保持原始坐标）
        self.waypoints = []
        self.waypoints.append(Pose(Point(0.4 ,0.4 ,0), self.quaternions[0]))
        self.waypoints.append(Pose(Point(3.902, 0.466, 0), self.quaternions[1]))
        self.waypoints.append(Pose(Point(3.683, 1.572, 0), self.quaternions[2]))
        self.waypoints.append(Pose(Point(3.040, 1.330, 0), self.quaternions[3]))
        self.waypoints.append(Pose(Point(2.840, 1.897, 0), self.quaternions[4]))
        self.waypoints.append(Pose(Point(2.276, 1.793, 0), self.quaternions[5]))
        self.waypoints.append(Pose(Point(2.324, 3.870, 0), self.quaternions[6]))
        self.waypoints.append(Pose(Point(1.053, 3.704, 0), self.quaternions[7]))
        self.waypoints.append(Pose(Point(1.190, 3.098, 0), self.quaternions[8]))
        self.waypoints.append(Pose(Point(0.993, 0.259, 0), self.quaternions[9]))
        self.waypoints.append(Pose(Point(0.516, 0.345, 0), self.quaternions[10]))

        # 状态变量（保持与你原脚本相同的 count 逻辑）
        self.count = 1
        self.stop_flag = 0

        # move_base action client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        if not self.move_base.wait_for_server(rospy.Duration(60)):
            rospy.logerr("move_base action server not available. Exiting.")
            rospy.signal_shutdown("move_base unavailable")
            return

        # clear_costmaps service（可选）
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=10)
            self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        except rospy.ROSException:
            rospy.logwarn("/move_base/clear_costmaps service not available.")
            self.clear_costmaps_service = None

        # recognition services（尽量等待，但允许超时）
        try:
            rospy.wait_for_service('recognize_person', timeout=10)
            rospy.wait_for_service('recognize_plate', timeout=10)
            self.client_person = rospy.ServiceProxy("recognize_person", detect)
            self.client_plate = rospy.ServiceProxy("recognize_plate", detect)
        except rospy.ROSException:
            rospy.logwarn("Recognition services not ready at startup; calls will be attempted later.")
            self.client_person = None
            self.client_plate = None

        rospy.loginfo("mission start!")
        self.main_loop()

    def main_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            while self.stop_flag != 1 and not rospy.is_shutdown():
                rospy.loginfo("正在前往目标点 %d", self.count)
                # 构造 goal（注意原脚本使用 waypoints[self.count]）
                try:
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    # 安全索引：如果超出范围就结束
                    if self.count < 0 or self.count >= len(self.waypoints):
                        rospy.logwarn("count 超出航点范围 (%d)，结束任务。", self.count)
                        self.stop_flag = 1
                        break
                    goal.target_pose.pose = self.waypoints[self.count]
                except Exception as e:
                    rospy.logerr("构造 goal 时出错: %s", str(e))
                    self.stop_flag = 1
                    break

                if self.move(goal):
                    # 到达成功后清除代价地图（若可用）
                    if self.clear_costmaps_service is not None:
                        try:
                            self.clear_costmaps_service()
                        except Exception:
                            rospy.logwarn("清除代价地图失败（被捕获），继续。")

                    self.count += 1
                    if self.count == 11:
                        self.stop_flag = 1
                        rospy.loginfo("达到 count==11, 停止任务。")
                    elif self.count == 4:
                        self.call_person_service(1)
                    elif self.count == 5:
                        self.call_person_service(2)
                    elif self.count == 9:
                        self.call_plate_service(3)
                else:
                    rospy.logwarn("移动失败或超时，跳过当前目标并继续下一个。")
                    self.count += 1
                    if self.count >= 11:
                        self.stop_flag = 1
                        rospy.loginfo("达到或超过最后航点，停止任务。")
            rate.sleep()
            if self.stop_flag == 1:
                break
        rospy.loginfo("主循环结束。")

    def move(self, goal):
        """发送 goal 并等待结果，超时或失败则返回 False"""
        try:
            self.move_base.send_goal(goal)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                return False
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    return True
                else:
                    rospy.loginfo("Goal fail or canceled, state=%d", state)
                    return False
        except Exception as e:
            rospy.logerr("与 move_base 交互发生异常: %s\n%s", str(e), traceback.format_exc())
            return False

    def call_person_service(self, arg):
        """安全地调用 recognize_person 服务"""
        if self.client_person is None:
            try:
                rospy.wait_for_service('recognize_person', timeout=5)
                self.client_person = rospy.ServiceProxy("recognize_person", detect)
            except rospy.ROSException:
                rospy.logwarn("recognize_person 服务暂不可用，跳过调用。")
                return
        try:
            resp = self.client_person(arg)
            rospy.sleep(1)
            rospy.loginfo("第一处社区识别到：%s", resp)
        except Exception as e:
            rospy.logwarn("调用 recognize_person 失败: %s", str(e))

    def call_plate_service(self, arg):
        """安全地调用 recognize_plate 服务"""
        if self.client_plate is None:
            try:
                rospy.wait_for_service('recognize_plate', timeout=5)
                self.client_plate = rospy.ServiceProxy("recognize_plate", detect)
            except rospy.ROSException:
                rospy.logwarn("recognize_plate 服务暂不可用，跳过调用。")
                return
        try:
            resp = self.client_plate(arg)
            rospy.sleep(1)
            rospy.loginfo("识别到的车牌号为：%s", resp)
        except Exception as e:
            rospy.logwarn("调用 recognize_plate 失败: %s", str(e))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        try:
            self.move_base.cancel_goal()
        except Exception:
            pass
        rospy.sleep(1)
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
