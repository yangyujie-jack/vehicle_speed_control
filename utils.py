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


def log_config(config):
    log = "Simulation:\n"
    log += f"\tname: {config.simulation.simulation_name}\n"
    if config.simulation.simulation_name == "const_v":
        log += f"\ttarget speed: {config.simulation.const_v}km/h\n"
    log += f"\tslope amplitude: {config.simulation.slope_amplitude}°\n"
    log += "Controller:\n"
    log += f"\tname: {config.controller.name}\n"
    if config.controller.name == "LQR":
        log += f"\tLQR Q2: {config.controller.lqr.Q2}\n"
    log += "Constants:\n"
    log += f"\tdt: {config.const.dt}s\n"
    print(log)