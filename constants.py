BICYCLE_LENGTH = 2
T_STEP = 0.1
N_HORIZON = 20
SIM_TIME = 5
POS_GAIN = 200
THETA_GAIN = 5
# DELTA_GAIN = 0.5

MAX_ERRORS = {
    "x": 0.5,
    "y": 0.4,
    "theta": 0.1,
}


# pygame constants
RATIO = 10 # Scaling factor for the screen
START_STATE = [800 / RATIO, 400 / RATIO, 0.0, 0.0]