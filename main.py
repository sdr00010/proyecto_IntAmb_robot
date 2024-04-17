#!/usr/bin/env pybricks-micropython

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
import time
from movimiento import avanzar as avn

# ---------------------------------------------------------------------------------------------------------------------------------

# Crear el objeto del robot.
robot_LEGO = EV3Brick()
left_motor = Motor(Port.A)  # motor rueda izquierda
right_motor = Motor(Port.D) # motor rueda derecha

# Prueba: avanzar hacia delante
avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 250)

wait(1000)

# Prueba: avanzar hacia atrás
avn.avanzar_atras(robot_LEGO, left_motor, right_motor, 250)

""" 
# 1. Make a sound.
ev3.speaker.beep()

# 2. Initialize a motor at port B.
test_motor = Motor(Port.A)
# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
test_motor.run_target(500, 90)

# 3. Play another beep sound.
ev3.speaker.beep(frequency=1000, duration=500) 
"""
