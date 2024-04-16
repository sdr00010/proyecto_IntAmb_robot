# This program requires LEGO EV3 MicroPython v2.0 or higher

# imports
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# imports locales
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './movimiento')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './comunicacion')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './interno')))

# ---------------------------------------------------------------------------------------------------------------------------------

""" # Crear el objeto del robot.
robot_LEGO = EV3Brick()


# 1. Make a sound.
ev3.speaker.beep()

# 2. Initialize a motor at port B.
test_motor = Motor(Port.A)
# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
test_motor.run_target(500, 90)

# 3. Play another beep sound.
ev3.speaker.beep(frequency=1000, duration=500) """
