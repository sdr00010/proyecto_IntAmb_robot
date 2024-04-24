#!/usr/bin/env pybricks-micropython

# import
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
# imports locales
from interno import utils

# funciones: avanzar hacia delante
def avanzar_adelante(robot: EV3Brick, left_motor: Motor, right_motor: Motor, distancia: float):
    # Inicializar el DriveBase
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=utils.CONFIG["parametros"]["wheel_diameter"], axle_track=utils.CONFIG["parametros"]["axle_track"])
    # Avanzar 20cm
    robot_db.straight(distancia)  # milímetros que avanza
    # pitar
    robot.speaker.beep()

# funciones: avanzar hacia atrás
def avanzar_atras(robot: EV3Brick, left_motor: Motor, right_motor: Motor, distancia: float):
    # Inicializar el DriveBase
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=utils.CONFIG["parametros"]["wheel_diameter"], axle_track=utils.CONFIG["parametros"]["axle_track"])
    # Avanzar 20cm hacia atrás
    robot_db.straight((distancia)*(-1))  # milímetros que retrocede
    # pitar
    robot.speaker.beep()

# funciones: avanzar N casillas hacia delante
def adelante_N_casillas(robot: EV3Brick, left_motor: Motor, right_motor: Motor, n: int):
    avanzar_adelante(robot, left_motor, right_motor, n*utils.CONFIG["parametros"]["medida_casilla"])

# funciones: avanzar N casillas hacia atrás
def atras_N_casillas(robot: EV3Brick, left_motor: Motor, right_motor: Motor, n: int):
    avanzar_atras(robot, left_motor, right_motor, n*utils.CONFIG["parametros"]["medida_casilla"])