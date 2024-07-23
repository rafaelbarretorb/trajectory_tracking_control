import math


def wrap_to_2pi(theta):
    """Wraps theta angle (radians) to the interval [0, 2*pi]."""
    return math.atan2(math.sin(theta - math.pi), math.cos(theta - math.pi)) + math.pi


def wrap_to_pi(theta):
    """Wraps theta angle (radians) to the interval [-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))
