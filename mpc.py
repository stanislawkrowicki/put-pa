"""MPC for controlling the docking procedure of a bicycle model"""
import numpy as np

import matplotlib.pyplot as plt
import do_mpc
import casadi as ca

from model import BicycleModel
import constants

class BicycleMPC(do_mpc.controller.MPC):
    """Model Predictive Controller with objective function and constraints for Bicycle model"""

    def __init__(
        self,
        model: do_mpc.model.Model,
    ) -> None:

        super().__init__(model)

        self.set_param(
            n_horizon=constants.N_HORIZON,
            t_step=constants.T_STEP,
            n_robust=1,
            store_full_solution=True,
            nlpsol_opts={"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0},
        )

        self.bounds["lower", "_x", "x_pos"] = 0
        self.bounds["upper", "_x", "y_pos"] = 160

        self.bounds["lower", "_x", "y_pos"] = 0
        self.bounds["upper", "_x", "y_pos"] = 80

        self.bounds["lower", "_u", "delta"] = -ca.pi / 4
        self.bounds["upper", "_u", "delta"] = ca.pi / 4

        self.bounds["lower", "_u", "v"] = -5
        self.bounds["upper", "_u", "v"] = 5

        # norm = lambda x, min, max: (x - min) / (max - min)
        lterm = (
            ((model.x["x_pos"] - model.tvp["set_x_pos"]) / 20 * constants.POS_GAIN) ** 2
            + ((model.x["y_pos"] - model.tvp["set_y_pos"]) / 20 * constants.POS_GAIN) ** 2
            + ((model.x["theta"] - model.tvp["set_theta"]) * constants.THETA_GAIN) ** 2
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

# x0 = np.array([5.0, 5.0, 0.0]).reshape(-1, 1)

# mpc.choose_setpoint(SETPOINT[0], SETPOINT[1], SETPOINT[2])
# simulator.x0 = x0
# mpc.x0 = x0

# mpc.set_initial_guess()

x_history = []
y_history = []
theta_history = []
delta_history = []
v_history = []


# for _ in range(int(SIM_TIME * 1 / T_STEP)):

#     u0 = bicycle_mpc.make_step(x0)
#     x0 = simulator.make_step(u0)
#     # x0 = simulate(x0, u0[0], u0[1])

#     x_history.append(x0[0])
#     y_history.append(x0[1])
#     theta_history.append(x0[2])
#     delta_history.append(u0[1])
#     v_history.append(u0[0])

model = None 
simulator = None 
bicycle_mpc = None 

def prepare_mpc():
    """Prepare the MPC controller"""
    global model, simulator, bicycle_mpc
    model = BicycleModel(L=constants.BICYCLE_LENGTH)

    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step=constants.T_STEP)
    simulator.set_tvp_fun(lambda t_now: simulator.get_tvp_template())
    simulator.setup()

    bicycle_mpc = BicycleMPC(model)

def set_simulation_target(curr_x, curr_y, curr_delta, target_x, target_y, target_delta):
    """Dock the car to the desired position"""
    global model, simulator, bicycle_mpc
    x0 = np.array([curr_x, curr_y, curr_delta]).reshape(-1, 1)
    bicycle_mpc.choose_setpoint(target_x, target_y, target_delta)

    simulator.x0 = x0
    bicycle_mpc.x0 = x0
    bicycle_mpc.set_initial_guess()

# plt.subplot(5, 1, 1)
# plt.plot(x_history, y_history)
# plt.title("Trajectory of the car")
# plt.xlabel("X position")
# plt.ylabel("Y position")
# plt.subplot(5, 1, 2)
# plt.plot(theta_history)
# plt.title("Orientation of the car")
# plt.subplot(5, 1, 3)
# plt.plot(delta_history)
# plt.title("Steering angle of the car")
# plt.subplot(5, 1, 4)
# plt.plot(v_history)
# plt.title("Velocity of the car")
# plt.show()
