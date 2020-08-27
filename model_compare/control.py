

def get_control(name, t):
    if name == "accel_then_brake":
        return _accel_then_brake(t)
    elif name == "brake_from_80":
        return _brake_from_80(t)
    elif name == "brake_from_120":
        return _brake_from_120(t)
    elif name == "full_throttle":
        return _full_throttle(t)
    elif name == "ramp_to_025":
        return _ramp_to_025(t)
    elif name == "ramp_to_050":
        return _ramp_to_050(t)


def _accel_then_brake(t):
    # alpha
    if t < 10:
        alpha = 0.65/10*t
    elif t < 10.5:
        alpha = 0.65-0.65/0.5*(t-10)
    else:
        alpha = 0
    # Pb
    if t < 15:
        Pb = 0
    elif t < 15.5:
        Pb = 0.8/0.5*(t-15)
    else:
        Pb = 0.8
    return alpha, Pb


def _brake_from_80(t):
    alpha = 0
    Pb = 3
    return alpha, Pb


def _brake_from_120(t):
    alpha = 0
    Pb = 3
    return alpha, Pb


def _full_throttle(t):
    if t < 0.1:
        alpha = 1/0.1*t
    else:
        alpha = 1
    Pb = 0
    return alpha, Pb


def _ramp_to_025(t):
    if t < 0.1:
        alpha = 0.25/0.1*t
    else:
        alpha = 0.25
    Pb = 0
    return alpha, Pb


def _ramp_to_050(t):
    if t < 0.1:
        alpha = 0.5/0.1*t
    else:
        alpha = 0.5
    Pb = 0
    return alpha, Pb