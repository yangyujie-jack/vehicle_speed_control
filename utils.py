from controller import *


def get_controller(name, config):
    if name == "LQR":
        return LQRController(config)
    elif name == "PID":
        return PIDController(config)
    elif name == "MPC":
        return MPCController(config)
    else:
        raise ValueError(f"Invalid controller name '{name}'")


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


def print_res(v_error, fuel_csp):
    print("")
    print(f"max velocity error {format(max(abs(v_error))*3.6, '.4f')} km/h")
    print(f"average velocity error {format(sum(abs(v_error))/len(v_error)*3.6, '.4f')} km/h")
    print(f"fuel consumption {format(fuel_csp, '.4f')} kg")