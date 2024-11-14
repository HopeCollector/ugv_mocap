#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from mpc import calc_cmd
from state_est import PoseFilter
import numpy as np
import yaml

# 读取参数
with open("/data/run.yml", "r") as file:
    params = yaml.safe_load(file)["controller"]

MAX_SPEED = params["max_speed"]
ODOM_TOPIC = params["feedback_odom_topic"]
print(f"load params: {params}")


class TrackerNode:
    __msg_real_path: Path
    __cur_state: Odometry
    __pub_cmd: rospy.Publisher
    __sub_odom: rospy.Subscriber
    __sub_tgt: rospy.Subscriber

    def __init__(self):
        rospy.init_node("tracker_node")
        self.__tgt = np.zeros(6)
        self.__msg_real_path = Path()
        self.__cur_state = np.zeros(6)
        self.__sub_odom = rospy.Subscriber(
            ODOM_TOPIC, Odometry, self.__cb_odom, queue_size=10
        )
        self.__sub_tgt = rospy.Subscriber(
            "car_target", Odometry, self.__cb_tgt, queue_size=1
        )
        self.__pub_real_path = rospy.Publisher("/out/path/real", Path, queue_size=2)
        self.__pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        self.__is_get_tgt = False
        self.__msg_real_path.header.frame_id = "odom"
        self.__pose_filter = PoseFilter(rospy.Time.now().to_nsec())
        rospy.Timer(rospy.Duration(0.01), self.__cb_ctrl)

    # 更新小车目标位姿
    def __cb_tgt(self, msg: Odometry):
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = 2 * np.arctan2(z, w)
        while yaw - self.__cur_state[2] > np.pi:
            yaw -= 2 * np.pi
        while yaw - self.__cur_state[2] < -np.pi:
            yaw += 2 * np.pi
        self.__tgt = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            ]
        )
        self.__is_get_tgt = True

    # 更新小车当前位姿
    def __cb_odom(self, msg: Odometry):
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = 2 * np.arctan2(z, w)

        # 使用 KF 进行状态估计
        # self.__cur_state = self.__pose_filter.update(
        #     [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw],
        #     rospy.Time.now().to_nsec(),
        # ).reshape((6,))

        # 直接使用 odom 数据作为小车当前位姿
        self.__cur_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        ])

        # 发布用于可视化的小车路径
        p = PoseStamped()
        p.header = msg.header
        p.header.frame_id = "odom"
        p.pose.position = msg.pose.pose.position
        p.pose.orientation = msg.pose.pose.orientation
        self.__msg_real_path.poses.append(p)
        self.__pub_real_path.publish(self.__msg_real_path)

    # 定时计算小车控制量
    def __cb_ctrl(self, event):
        if not self.__is_get_tgt:
            return
        vl, va = calc_cmd(self.__tgt, self.__cur_state)
        cmd = Twist()
        cmd.linear.x = np.clip(vl, -MAX_SPEED, MAX_SPEED)
        cmd.angular.z = va
        self.__pub_cmd.publish(cmd)


if __name__ == "__main__":
    node = TrackerNode()
    rospy.spin()
