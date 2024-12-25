import do_mpc
import threading
import constants
import numpy as np
import model
import mpc

class DockingAction:
    _current_state = [] # [x, y, delta, v]
    _current_pos = [] # [x, y, delta]
    _target_pos = [] # [x, y, delta]
    _stop_event = False
    _is_docking = False

    @classmethod
    def _dock(cls):
        cls._is_docking = True
        mpc.set_simulation_target(cls._current_pos[0], cls._current_pos[1], cls._current_pos[2], 
                                  cls._target_pos[0], cls._target_pos[1], cls._target_pos[2])

        x0 = cls._current_pos
        errors = cls._target_pos - x0.flatten()

        while np.any(np.abs(errors) > constants.MAX_ERROR):
            if cls._stop_event:
                cls._stop_event = False
                cls._is_docking = False
                break
            u0 = mpc.bicycle_mpc.make_step(x0)
            x0 = mpc.simulator.make_step(u0)
            x_list = x0.tolist()
            u_list = u0.tolist()
            cls._current_state = [x_list[0][0], x_list[1][0], x_list[2][0], u_list[0][0]]
            errors = cls._target_pos - x0.flatten()
            print(errors)
        
        cls._is_docking = False


    @classmethod
    def run_docking_thread(cls, current_x, current_y, current_delta, target_x, target_y, target_delta):
        if cls._is_docking:
            cls.stop_docking()

        while cls._stop_event:
            pass

        cls._current_pos = np.array([current_x, current_y, current_delta]).reshape(-1, 1)
        cls._current_state = [current_x, current_y, current_delta, 0]
        cls._target_pos = np.array([target_x, target_y, target_delta])

        threading.Thread(target=cls._dock).start()


    @classmethod
    def get_current_state(cls):
        return cls._current_state
    

    @classmethod
    def stop_docking(cls):
        if cls._is_docking:
            cls._stop_event = True 


    @classmethod
    def is_docking(cls):
        return cls._is_docking