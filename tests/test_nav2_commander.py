"""Тесты автомата режима Nav2 (взлёт -> следование -> посадка -> force-disarm).

Проверяют полную последовательность и, главное, что force-disarm выдаётся
только ПОСЛЕ подтверждения касания земли по дальномеру (dist_bottom).
"""
import numpy as np

from _common import ROOT  # noqa: F401
from uav_offboard.nav2_commander import (
    NAV_STATE_AUTO_LAND,
    NAV_STATE_OFFBOARD,
    Nav2Command,
    Nav2Commander,
    Nav2State,
)


def _simulate(commander, steps=6000, move_until=8.0):
    """Прогон автомата с примитивной «физикой» и программной скоростью.

    В режиме FOLLOW до момента ``move_until`` задаётся скорость вперёд,
    затем — остановка, что должно привести к автопосадке и force-disarm.
    """
    armed = False
    nav = 0
    pos = np.zeros(3)
    t = 0.0
    dt = commander.dt
    seq = []

    for _ in range(steps):
        if commander.state is Nav2State.FOLLOW and t < move_until:
            commander.set_velocity_cmd(0.3, 0.0, 0.0)
        else:
            commander.set_velocity_cmd(0.0, 0.0, 0.0)

        # Дальномер: считаем, что высота и есть дистанция до земли (валидна).
        dist_bottom = float(pos[2])
        r = commander.step(armed, nav, pos, t, dist_bottom)

        for cmd in r.commands:
            seq.append(cmd)
            if cmd is Nav2Command.ARM:
                armed = True
            elif cmd is Nav2Command.DISARM:
                armed = False
            elif cmd is Nav2Command.SET_OFFBOARD:
                nav = NAV_STATE_OFFBOARD
            elif cmd is Nav2Command.LAND:
                nav = NAV_STATE_AUTO_LAND

        # «Физика»: при посадке снижаемся, иначе — догоняем setpoint.
        if Nav2Command.LAND in r.commands:
            pos[2] = max(0.0, pos[2] - 0.05)
        elif r.setpoint is not None:
            pos += np.clip(r.setpoint - pos, -0.05, 0.05)

        if commander.finished:
            break
        t += dt

    return armed, nav, pos, t, seq


def test_follow_then_force_disarm_sequence():
    c = Nav2Commander(takeoff_height=2.0, dt=0.1, stop_timeout=1.0,
                      touch_debounce=0.3)
    armed, nav, pos, t, seq = _simulate(c, move_until=8.0)

    assert c.state is Nav2State.DONE, f'не завершена за {t:.1f}с'
    assert not armed, 'должна быть снята с охраны (disarm)'

    names = [x.name for x in seq]
    for expected in ('ARM', 'SET_OFFBOARD', 'LAND', 'DISARM'):
        assert expected in names, f'нет команды {expected}'
    # порядок: ARM -> SET_OFFBOARD -> LAND -> DISARM
    order = [names.index(x) for x in ('ARM', 'SET_OFFBOARD', 'LAND', 'DISARM')]
    assert order == sorted(order), f'нарушен порядок команд: {names}'

    # Дрон реально двигался вперёд (следование за cmd_vel), затем сел.
    assert pos[0] > 0.2, f'дрон не двигался по X: {pos[0]:.2f}'
    assert pos[2] < 0.12, f'дрон не сел: alt={pos[2]:.2f}'

    # Дизарм произошёл только после касания (alt почти нулевая).
    disarm_idx = names.index('DISARM')
    assert disarm_idx > names.index('LAND'), 'DISARM раньше LAND'


def test_disarm_only_after_touch_confirmation():
    c = Nav2Commander(takeoff_height=2.0, dt=0.1, touch_tol=0.12,
                      touch_debounce=0.3)
    # Доводим автомат напрямую до LANDING (взлёт/следование не интересны).
    c.state = Nav2State.LANDING
    c._state_start = 0.0
    armed = True
    nav = NAV_STATE_AUTO_LAND
    pos = np.array([0.0, 0.0, 1.0])  # всё ещё в воздухе
    t = 0.0

    # На высоте 1 м касания нет -> только LAND, никакого DISARM.
    r = c.step(armed, nav, pos, t, dist_bottom=1.0)
    assert Nav2Command.LAND in r.commands
    assert Nav2Command.DISARM not in r.commands
    assert c.state is Nav2State.LANDING

    # Подтверждённое касание: держим dist_bottom < touch_tol на протяжении
    # debounce. После этого — переход в DISARMING и выдача DISARM.
    for _ in range(5):
        r = c.step(armed, nav, pos, t, dist_bottom=0.05)
        t += 0.1

    assert c.state is Nav2State.DISARMING
    assert Nav2Command.DISARM in r.commands


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
