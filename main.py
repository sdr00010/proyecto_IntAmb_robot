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
from interno import utils
from movimiento import avanzar as avn

# ---------------------------------------------------------------------------------------------------------------------------------

# Crear el objeto del robot.
robot_LEGO = EV3Brick()
left_motor = Motor(Port.A)  # motor rueda izquierda
right_motor = Motor(Port.D) # motor rueda derecha

# ---------------------------------------------------------------------------------------------------------------------------------


# MAIN DE PRUEBAS

# Prueba: avanzar 1 casillas hacia delante
avn.adelante_N_casillas(robot_LEGO, left_motor, right_motor, 1)

robot_db = DriveBase(left_motor, right_motor, wheel_diameter=utils.CONFIG["parametros"]["wheel_diameter"], axle_track=utils.CONFIG["parametros"]["axle_track"])
robot_db.turn(90)

# Prueba: avanzar 2 casillas hacia delante
avn.adelante_N_casillas(robot_LEGO, left_motor, right_motor, 2)

robot_db = DriveBase(left_motor, right_motor, wheel_diameter=utils.CONFIG["parametros"]["wheel_diameter"], axle_track=utils.CONFIG["parametros"]["axle_track"])
robot_db.turn(90)

avn.adelante_N_casillas(robot_LEGO, left_motor, right_motor, 1)

avn.atras_N_casillas(robot_LEGO, left_motor, right_motor, 1)

