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


def print_res(v_error, fuel_csp):
    print("")
    print(f"max velocity error {format(max(abs(v_error))*3.6, '.4f')} km/h")
    print(f"average velocity error {format(sum(abs(v_error))/len(v_error)*3.6, '.4f')} km/h")
    print(f"fuel consumption {format(fuel_csp, '.4f')} kg")
