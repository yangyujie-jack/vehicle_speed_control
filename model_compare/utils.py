


def load_carsim_data(path):
    vs = []
    with open(path, 'r') as f:
        collecting = False
        for line in f.readlines():
            if not collecting:
                if "Time" in line and "Vx" in line:
                    collecting = True
            else:
                if "Time" in line and "VxTarget" in line:
                    break
                if line == '\n':
                    continue
                [t, v] = line.split('\t')
                v = float(v)
                vs.append(v)
    return vs


def get_config(name):
    if name == "accel_then_brake":
        v0, T = 0, 30
    elif name == "brake_from_80":
        v0, T = 80/3.6, 8
    elif name == "brake_from_120":
        v0, T = 120/3.6, 8
    elif name == "full_throttle":
        v0, T = 0, 10
    elif name == "ramp_to_025":
        v0, T = 0, 10
    elif name == "ramp_to_050":
        v0, T = 0, 10
    return v0, T

