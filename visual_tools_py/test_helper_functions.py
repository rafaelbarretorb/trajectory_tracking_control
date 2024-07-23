import math
import pytest

from helper_functions import wrap_to_pi, wrap_to_2pi


def test_wrap_to_pi():
    # x axis positive
    assert wrap_to_pi(math.radians(360)) == pytest.approx(math.radians(0.0))

    # y axis positive
    assert wrap_to_pi(math.radians(-270)) == pytest.approx(math.radians(90.0))

    # x axis negative
    assert wrap_to_pi(math.radians(-540)) == pytest.approx(math.radians(-180.0))
    assert wrap_to_pi(math.radians(540)) == pytest.approx(math.radians(180.0))

    # y axis negative
    assert wrap_to_pi(math.radians(270)) == pytest.approx(math.radians(-90.0))

    # 1st quadrant
    assert wrap_to_pi(math.radians(-270 - 45)) == pytest.approx(math.radians(45.0))

    # 2nd quadrant
    assert wrap_to_pi(math.radians(-180 - 45)) == pytest.approx(math.radians(90.0 + 45.0))

    # 3rd quadrant
    assert wrap_to_pi(math.radians(180 + 45)) == pytest.approx(math.radians(-180.0 + 45.0))

    # 4th quadrant
    assert wrap_to_pi(math.radians(270 + 45)) == pytest.approx(math.radians(-90.0 + 45.0))


def test_wrap_to_2pi():
    # x axis positive
    assert wrap_to_2pi(math.radians(720)) == pytest.approx(math.radians(360.0))
    assert wrap_to_2pi(math.radians(0)) == pytest.approx(math.radians(0.0))

    # y axis positive
    assert wrap_to_2pi(math.radians(-270)) == pytest.approx(math.radians(90.0))

    # x axis negative
    assert wrap_to_2pi(math.radians(-540)) == pytest.approx(math.radians(180.0))

    # y axis negative
    assert wrap_to_2pi(math.radians(-90)) == pytest.approx(math.radians(270.0))

    # 1st quadrant
    assert wrap_to_2pi(math.radians(-270 - 45)) == pytest.approx(math.radians(45.0))

    # 2nd quadrant
    assert wrap_to_2pi(math.radians(-180 - 45)) == pytest.approx(math.radians(90.0 + 45.0))

    # 3rd quadrant
    assert wrap_to_2pi(math.radians(-720 + 180 + 45)) == pytest.approx(math.radians(180.0 + 45.0))

    # 4th quadrant
    assert wrap_to_2pi(math.radians(720 + 270 + 45)) == pytest.approx(math.radians(270.0 + 45.0))
