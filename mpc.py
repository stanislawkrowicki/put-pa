"""Node for controlling the docking procedure of the F1/10th car"""

import time
import threading
import numpy as np

from bicycle_model import BicycleModel
from bicycle_mpc import BicycleMPC
import matplotlib.pyplot as plt
import do_mpc
import casadi as ca


class BicycleModel(do_mpc.model.Model):
    """Bicycle model for the F1/10th car"""

    def __init__(self, L: float) -> None:
        super().__init__("continuous")

        self.set_variable(var_type="_x", var_name="x_pos", shape=(1, 1))
        self.set_variable(var_type="_x", var_name="y_pos", shape=(1, 1))
        self.set_variable(var_type="_x", var_name="theta", shape=(1, 1))

        self.set_variable(var_type="_u", var_name="v")
        self.set_variable(var_type="_u", var_name="delta")

        self.set_variable(var_type="_tvp", var_name="set_x_pos")
        self.set_variable(var_type="_tvp", var_name="set_y_pos")
        self.set_variable(var_type="_tvp", var_name="set_theta")

        d_x_pos = self.u["v"] * ca.cos(self.x["theta"])
        d_y_pos = self.u["v"] * ca.sin(self.x["theta"])
        d_theta = self.u["v"] * ca.tan(self.u["delta"]) / L

        self.set_rhs("x_pos", d_x_pos)
        self.set_rhs("y_pos", d_y_pos)
        self.set_rhs("theta", d_theta)

        self.setup()


class BicycleMPC(do_mpc.controller.MPC):
    """Model Predictive Controller with objective function and constraints for the F1/10th car"""

    def __init__(
        self,
        model: do_mpc.model.Model,
    ) -> None:

        super().__init__(model)

        self.set_param(
            n_horizon=N_HORIZON,
            t_step=T_STEP,
            n_robust=1,
            store_full_solution=True,
            nlpsol_opts={"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0},
        )

        POS_GAIN = 40
        THETA_GAIN = 1
        DELTA_GAIN = 0.5

        self.bounds["lower", "_x", "x_pos"] = 0
        self.bounds["upper", "_x", "y_pos"] = 20

        self.bounds["lower", "_x", "y_pos"] = 0
        self.bounds["upper", "_x", "y_pos"] = 20

        self.bounds["lower", "_u", "delta"] = -ca.pi / 4
        self.bounds["upper", "_u", "delta"] = ca.pi / 4

        self.bounds["lower", "_u", "v"] = -5
        self.bounds["upper", "_u", "v"] = 5

        norm = lambda x, min, max: (x - min) / (max - min)
        lterm = (
            ((model.x["x_pos"] - model.tvp["set_x_pos"]) / 20 * POS_GAIN) ** 2
            + ((model.x["y_pos"] - model.tvp["set_y_pos"]) / 20 * POS_GAIN) ** 2
            + ((model.x["theta"] - model.tvp["set_theta"]) * THETA_GAIN) ** 2
        )

        mterm = lterm
        self.set_objective(mterm=mterm, lterm=lterm)
        self.set_rterm(v=1e-2, delta=1e-2)

        self.tvp_template = self.get_tvp_template()
        self.set_tvp_fun(lambda _: self.tvp_template)
        self.setup()

    def choose_setpoint(self, x_pos: float, y_pos: float, theta: float) -> None:
        """Choose setpoint for the MPC controller"""
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_x_pos"] = x_pos
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_y_pos"] = y_pos
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_theta"] = theta


L = 2
T_STEP = 0.2
N_HORIZON = 20
SIM_TIME = 10
SETPOINT = [15, 7, -0.4]

model = BicycleModel(L=L)

simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=T_STEP)
simulator.set_tvp_fun(lambda t_now: simulator.get_tvp_template())
simulator.setup()


x0 = np.array([5.0, 5.0, 0.0]).reshape(-1, 1)
x2 = np.array([5.0, 5.0, 0.0]).reshape(-1, 1)
mpc = BicycleMPC(model)

mpc.choose_setpoint(SETPOINT[0], SETPOINT[1], SETPOINT[2])
simulator.x0 = x0
mpc.x0 = x0

mpc.set_initial_guess()

x_history = []
y_history = []
theta_history = []
delta_history = []
v_history = []


for _ in range(int(SIM_TIME * 1 / T_STEP)):

    u0 = mpc.make_step(x0)
    x0 = simulator.make_step(u0)
    # x0 = simulate(x0, u0[0], u0[1])

    x_history.append(x0[0])
    y_history.append(x0[1])
    theta_history.append(x0[2])
    delta_history.append(u0[1])
    v_history.append(u0[0])


plt.subplot(5, 1, 1)
plt.plot(x_history, y_history)
plt.title("Trajectory of the car")
plt.xlabel("X position")
plt.ylabel("Y position")
plt.subplot(5, 1, 2)
plt.plot(theta_history)
plt.title("Orientation of the car")
plt.subplot(5, 1, 3)
plt.plot(delta_history)
plt.title("Steering angle of the car")
plt.subplot(5, 1, 4)
plt.plot(v_history)
plt.title("Velocity of the car")
plt.show()