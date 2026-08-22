"""Тесты автомата миссии (полный прогон с симуляцией кинематики)."""
import numpy as np

from _common import ROOT  # noqa: F401
from uav_offboard.mission_controller import (
    NAV_STATE_AUTO_LAND,
    NAV_STATE_OFFBOARD,
    Command,
    MissionController,
    MissionState,
)


def test_full_mission_sequence():
    mc = MissionController(
        takeoff_height=2.0, hover_time=1.0,
        waypoints=[[1.0, 0.0, 2.0]], dt=0.1,
        position_tol=0.1, land_tol=0.1)

    armed = False
    nav = 0
    pos = np.zeros(3)
    t = 0.0
    seq = []

    for _ in range(4000):
        r = mc.step(armed, nav, pos, t)
        for c in r.commands:
            seq.append(c)
            if c is Command.ARM:
                armed = True
            elif c is Command.DISARM:
                armed = False
            elif c is Command.SET_OFFBOARD:
                nav = NAV_STATE_OFFBOARD
            elif c is Command.LAND:
                nav = NAV_STATE_AUTO_LAND

        if r.setpoint is not None:
            pos += np.clip(r.setpoint - pos, -0.05, 0.05)
        if Command.LAND in r.commands:
            pos[2] = max(0.0, pos[2] - 0.05)

        if mc.finished:
            break
        t += 0.1

    assert mc.state is MissionState.DONE
    names = [c.name for c in seq]
    assert 'ARM' in names
    assert 'SET_OFFBOARD' in names
    assert 'LAND' in names
    assert 'DISARM' in names
    # порядок: ARM -> SET_OFFBOARD -> LAND -> DISARM
    order = [names.index(x) for x in ('ARM', 'SET_OFFBOARD', 'LAND', 'DISARM')]
    assert order == sorted(order)


def test_no_waypoints_goes_straight_to_land():
    mc = MissionController(takeoff_height=1.0, hover_time=0.5, waypoints=[])
    armed = False
    nav = 0
    pos = np.zeros(3)
    t = 0.0
    for _ in range(4000):
        r = mc.step(armed, nav, pos, t)
        if Command.ARM in r.commands:
            armed = True
        if Command.DISARM in r.commands:
            armed = False
        if Command.SET_OFFBOARD in r.commands:
            nav = NAV_STATE_OFFBOARD
        if Command.LAND in r.commands:
            nav = NAV_STATE_AUTO_LAND
            pos[2] = max(0.0, pos[2] - 0.1)
        if r.setpoint is not None:
            pos += np.clip(r.setpoint - pos, -0.05, 0.05)
        if mc.finished:
            break
        t += 0.1
    assert mc.state is MissionState.DONE


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
