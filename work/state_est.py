from filterpy.kalman import KalmanFilter
import numpy as np


class PoseFilter:
    """
    A class used to represent a Pose Filter using a Kalman Filter.
    Attributes
    ----------
    time : int
        The initial time for the filter.
    kf : KalmanFilter
        An instance of the KalmanFilter class used for state estimation.
    Methods
    -------
    __init__(self, time: int)
        Initializes the PoseFilter with the given time and sets up the Kalman Filter parameters.
    """

    def __init__(self, time: int):
        """
        Initialize the state estimation class with a given time and set up the Kalman Filter.
        Parameters:
            time (int): The initial time(ns) for the state estimation.
        """

        self.time = time

        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        # 初始化观测矩阵 H
        # z = Hx + v
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]]
        )

        # 初始化过程噪声协方差矩阵 Q
        # w ~ N(0, Q)
        self.kf.Q = np.eye(6) * 1

        # 初始化观测噪声协方差矩阵 R
        # v ~ N(0, R)
        self.kf.R = np.eye(3) * 1e-5

        # 初始化先验状态估计
        self.kf.x = np.array([[0, 0, 0, 0, 0, 0]]).T

        # 初始化先验估计协方差矩阵 P
        # P = E[(x - x_hat)(x - x_hat)^T]
        self.kf.P = np.eye(6)

    def update(self, z: np.ndarray, time: int) -> np.ndarray:
        dt = (time - self.time) / 1e9
        self.time = time

        # vx vy vyaw 是本地坐标系下的速度
        # dx = vx * cos(yaw) * dt - vy * sin(yaw) * dt
        # dy = vx * sin(yaw) * dt + vy * cos(yaw) * dt
        # x: [x, y, yaw, vx, vy, vyaw]

        yaw = self.kf.x.item(2)
        self.kf.F = np.array(
            [
                [1, 0, 0, np.cos(yaw) * dt, -np.sin(yaw) * dt, 0],
                [0, 1, 0, np.sin(yaw) * dt, np.cos(yaw) * dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )
        self.kf.predict()

        # 角度归一化
        yaw = self.kf.x[2]
        self.kf.x[2] = np.mod(yaw + np.pi, 2 * np.pi) - np.pi
        while z[2] - self.kf.x[2] > np.pi:
            z[2] -= 2 * np.pi
        while z[2] - self.kf.x[2] < -np.pi:
            z[2] += 2 * np.pi

        self.kf.update(z)
        return self.kf.x
