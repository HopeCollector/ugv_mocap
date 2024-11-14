from nav_msgs.msg import Odometry
import casadi as ca
import numpy as np
from math import sin, cos
from scipy.io import loadmat, savemat
from scipy.spatial.transform import Rotation as R
import os

MAT_FILE = "/data/mpc.mat"


def odom_to_yaw(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    return r.as_euler("zyx")[0]


# 中心对中心误差
def error_state(X_r, X_f) -> np.ndarray:
    x_r, y_r, theta_r = X_r.item(0), X_r.item(1), X_r.item(2)
    x_f, y_f, theta_f = X_f.item(0), X_f.item(1), X_f.item(2)
    mtx = np.array(
        [
            [cos(theta_f), sin(theta_f), 0.0],
            [-sin(theta_f), cos(theta_f), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    vec = np.array([[x_r - x_f], [y_r - y_f], [theta_r - theta_f]])
    return mtx @ vec


class mpc_controller:
    def __init__(self, matfile: str):
        cup_num = os.cpu_count()
        os.environ["OMP_NUM_THREADS"] = f"{cup_num * 2}"
        self.__filename: str = matfile
        self.__last_u: np.ndarray = np.zeros((2, 1))
        self.__load_data()
        self.__prob = ca.Opti()
        self.__u_f = self.__prob.parameter(2, 1)
        self.__u_r = self.__prob.parameter(2, 1)
        self.__x_e = self.__prob.parameter(3, 1)
        self.__x_es: list[ca.MX] = []
        self.__x_es_prv: list[np.ndarray] = []
        self.__us: list[ca.MX] = [self.__prob.variable(2, 1) for _ in range(self.N)]
        self.__us_prv: list[np.ndarray] = []
        self.ipopt_cfg = {
            "max_iter": 100,
            # "tol": 1e-6,
            # "acceptable_tol": 1e-8,
            # "acceptable_obj_change_tol": 1e-6,
            "print_level": 0,
            "sb": "yes",
            "linear_solver": "mumps",
        }
        self.solver_cfg = {
            "ipopt": self.ipopt_cfg,
            "print_time": False,
        }
        self.__build_prob()

    def __load_data(self):
        data = loadmat(self.__filename)
        self.N = data["N"].item()
        self.Q = data["Q"]
        self.R = data["R"]
        self.P = data["P"]
        self.K = data["K"]
        self.Omega = data["Omega"].item()
        self.X_u = data["X_u"]
        self.X_l = data["X_l"]
        self.U_u = data["U_u"]
        self.U_l = data["U_l"]
        self.Delta_U = data["Delta_U"]
        self.sample_time = data["sample_time"].item()
        self.rho = data["rho"].item()
        self.Final_Delta_U = data["Final_Delta_U"]
        self.L_phi = data["L_phi"].item()
        self.gamma = data["gamma"].item()

    def __next_x_e(self, x_e: ca.MX, u: ca.MX):
        v_f, w_f = u[0], u[1]
        v_r, w_r = self.__u_r[0], self.__u_r[1]

        def f(X: ca.MX) -> ca.MX:
            x_e, y_e, theta_e = X[0, 0], X[1, 0], X[2, 0]
            return ca.vertcat(
                -self.rho * w_r * ca.sin(theta_e)
                - v_f
                + v_r * ca.cos(theta_e)
                + w_f * y_e,
                self.rho * w_r * ca.cos(theta_e)
                + v_r * ca.sin(theta_e)
                + w_f * (-self.rho - x_e),
                -w_f + w_r,
            )

        k1 = f(x_e)
        k2 = f(x_e + self.sample_time / 2 * k1)
        k3 = f(x_e + self.sample_time / 2 * k2)
        k4 = f(x_e + self.sample_time * k3)
        x_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        return x_e + k1 * self.sample_time

    def __build_prob(self):
        # add cost
        self.__x_es = []
        x_e = self.__next_x_e(self.__x_e, self.__u_f)
        costs = []
        for i in range(self.N):
            ui = self.__us[i]
            x_e = self.__next_x_e(x_e, ui)
            costs.append(ui.T @ self.R @ ui)
            costs.append(x_e.T @ self.Q @ x_e)
            self.__x_es.append(x_e)
        costs.append(self.__x_es[-1].T @ self.P @ self.__x_es[-1])
        self.__prob.minimize(sum(costs))

        # add constraints
        for i in range(self.N):
            self.__prob.subject_to(self.__us[i] >= self.U_l)
            self.__prob.subject_to(self.__us[i] <= self.U_u)
            self.__prob.subject_to(
                self.__x_es[i]
                >= self.X_l + i * self.gamma * ca.exp(self.L_phi * (i - 1))
            )
            self.__prob.subject_to(
                self.__x_es[i]
                <= self.X_u - i * self.gamma * ca.exp(self.L_phi * (i - 1))
            )
            if i > 0:
                self.__prob.subject_to(self.__us[i] - self.__us[i - 1] <= self.Delta_U)
                self.__prob.subject_to(self.__us[i] - self.__us[i - 1] >= -self.Delta_U)
        self.__prob.subject_to(
            self.__x_es[-1].T @ self.P @ self.__x_es[-1] <= self.Omega
        )
        self.__prob.subject_to(
            self.K @ self.__x_es[-1] - self.__us[-1] >= -self.Final_Delta_U
        )
        self.__prob.subject_to(
            self.K @ self.__x_es[-1] - self.__us[-1] <= self.Final_Delta_U
        )
        self.__prob.solver("ipopt", self.solver_cfg)

    def __update_data(self, x_follower, u_follower, x_request, u_request):
        self.__prob.set_value(self.__u_f, u_follower)
        self.__prob.set_value(self.__u_r, u_request)
        self.__prob.set_value(self.__x_e, error_state(x_request, x_follower))
        if len(self.__us_prv) > 0:
            (
                self.__prob.set_initial(self.__us[i], self.__us_prv[i])
                for i in range(self.N)
            )

    def __clean_cache(self):
        self.__x_es_prv = []
        self.__us_prv = []

    def rebuild(self):
        self.__build_prob()

    def exit_state(self):
        return self.__exit_state

    def solve(self, x_follower, u_follower, x_request, u_request) -> np.ndarray:
        self.__update_data(x_follower, u_follower, x_request, u_request)
        self.__clean_cache()
        ret = np.zeros((2, 1))
        try:
            sol = self.__prob.solve()
            self.__exit_state = sol.stats()["return_status"]
            ret = sol.value(self.__us[0])
            (self.__us_prv.append(sol.value(self.__us[i]) for i in range(self.N)))
            (self.__x_es_prv.append(sol.value(self.__x_es[i]) for i in range(self.N)))
        except:
            ret = self.__prob.debug.value(self.__us[0])
            self.__exit_state = -87
            (
                self.__us_prv.append(self.__prob.debug.value(self.__us[i]))
                for i in range(self.N)
            )
            (
                self.__x_es_prv.append(self.__prob.debug.value(self.__us[i]))
                for i in range(self.N)
            )
        ret = np.reshape(ret, (-1, 1))
        delta_u = np.clip(ret - self.__last_u, -self.Delta_U, self.Delta_U)
        ret = self.__last_u + delta_u
        self.__last_u = ret
        return ret


ros_ctrl = mpc_controller(MAT_FILE)
# ros_ctrl.N = 10
ros_ctrl.sample_time = 0.1


def calc_cmd(tgt: np.ndarray, cur: np.ndarray):

    x_r = np.array([[tgt.item(0)], [tgt.item(1)], [tgt.item(2)]])
    u_r = np.array([[np.linalg.norm(tgt[3:5])], [tgt.item(5)]])
    x_f = np.array([[cur.item(0)], [cur.item(1)], [cur.item(2)]])
    u_f = np.array([[np.linalg.norm(cur[3:5])], [cur.item(5)]])
    u = ros_ctrl.solve(x_f, u_f, x_r, u_r)
    return u.item(0), u.item(1)


if __name__ == "__main__":
    x_r = np.array([[0], [1.0], [0]])
    u_r = np.array([[0.8], [0.2]])
    x_f = np.array([[0], [0], [0]])
    u_f = np.array([[1.0], [0.2]])
    ctrl = mpc_controller(MAT_FILE)

    u = ctrl.solve(x_f, u_f, x_r, u_r)
    print(u.T)
