import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml

# 读取参数
with open("/data/run.yml", "r") as file:
    params = yaml.safe_load(file)["target_publisher"]

DT = params["dt"]
RADDIUS = params["radius"]
LIN_SPEED = params["linear_speed"]
ANG_SPEED = LIN_SPEED / RADDIUS
DELTA = params["offset"]
print(f"load params: {params}")


def next_state(state):
    x, y, yaw = state
    vx = LIN_SPEED * np.cos(yaw)
    vy = LIN_SPEED * np.sin(yaw)
    return np.array([x + vx * DT, y + vy * DT, yaw + ANG_SPEED * DT])


class TargetPublisher:

    def __init__(self):
        self.sub_odom = rospy.Subscriber(
            "/qualisys/robotA/odom", Odometry, self.init_state, queue_size=10
        )

    def init_state(self, msg):
        # 停止订阅
        self.sub_odom.unregister()

        # 计算小车当前位姿
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = 2 * np.arctan2(z, w)
        init_state = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
            ]
        )
        delta = (
            np.array(
                [
                    [np.cos(yaw), -np.sin(yaw)],
                    [np.sin(yaw), np.cos(yaw)],
                ]
            )
            @ np.array([[0.0, DELTA]]).T
        )

        # 计算轨迹初始位姿
        self.state = init_state + np.array([delta.item(0), delta.item(1), 0.0])

        # 发布完整跑一圈的路径
        state = self.state
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()
        # 跑一圈的耗时：t = 2 * np.pi / ANG_SPEED
        # 跑一圈的计算步数：n = t / DT
        for i in range(int(np.ceil(2 * np.pi / ANG_SPEED / DT))):
            point = PoseStamped()
            point.header.frame_id = "odom"
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = state[0]
            point.pose.position.y = state[1]
            point.pose.orientation.z = np.sin(state[2] / 2)
            point.pose.orientation.w = np.cos(state[2] / 2)
            path.poses.append(point)
            state = next_state(state)
        self.path_publisher_.publish(path)
        self.path = path

        # 往前先走一点
        for i in range(50):
            self.state = next_state(self.state)

        # 设置发布器、定时器
        self.publisher_ = rospy.Publisher("car_target", Odometry, queue_size=10)
        self.ui_publisher_ = rospy.Publisher(
            "car_target_pose", PoseStamped, queue_size=10
        )
        self.path_publisher_ = rospy.Publisher("ref_path", Path, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(DT), self.timer_callback)
        self.timer_1s = rospy.Timer(rospy.Duration(1), self.t1s_cb)
        return

    # 每秒发布一次路径
    def t1s_cb(self, event):
        self.path.header.stamp = rospy.Time.now()
        self.path_publisher_.publish(self.path)

    # 每 DT 秒发布一次目标位姿
    def timer_callback(self, event):
        # 更新状态
        self.state = next_state(self.state)

        # 发布期望
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.orientation.z = np.sin(self.state[2] / 2)
        msg.pose.pose.orientation.w = np.cos(self.state[2] / 2)
        msg.twist.twist.linear.x = LIN_SPEED * 0.5
        msg.twist.twist.angular.z = ANG_SPEED
        self.publisher_.publish(msg)

        # 发布可视化用的标记
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ui_publisher_.publish(pose)


def main():
    rospy.init_node("target_publisher")
    car_controller = TargetPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
