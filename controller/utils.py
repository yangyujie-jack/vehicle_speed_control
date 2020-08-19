from controller import *


def get_controller(config):
    if config.controller.name == "LQR":
        return LQRController(config)
    elif config.controller.name == "PID":
        return PIDController(config)
    elif config.controller.name == "MPC":
        return MPCController(config)
    else:
        raise ValueError(f"Invalid controller name '{config.controller.name}'")
