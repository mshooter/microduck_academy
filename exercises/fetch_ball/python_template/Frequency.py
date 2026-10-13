"""
Frequency module from Robotics Academy exercises. 

Frequency.tick(hz) goes at the end of the student's main loop and holds the loop to roughly that rate.
"""
import time
from datetime import datetime

start_time_internal_freq_control = None


def tick(ideal_cycle: int = 50):
    """Hold the student's main loop to a fixed rate: call Frequency.tick(hz) at the end of each loop.

    Copied from JdeRobot's Robotics Academy, exercises/follow_line/python_template/Frequency.py
    (https://github.com/JdeRobot/RoboticsAcademy, branch humble-devel, commit ff8b1bf), only this
    docstring added. Kept identical so it behaves like every other Academy exercise.

    Licence: GNU GPL v3.0, as stated in the Robotics Academy repository. Unlike the rest of this
    repository (Apache 2.0), this file is under the GPL. See NOTICE.
    """
    global start_time_internal_freq_control
    finish_time = datetime.now()
    ideal_ms = 1000 / ideal_cycle

    if start_time_internal_freq_control is None:
        start_time_internal_freq_control = finish_time
        return

    dt = finish_time - start_time_internal_freq_control
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

    if ms < ideal_ms:
        time.sleep((ideal_ms - ms) / 1000.0)

    start_time_internal_freq_control = finish_time