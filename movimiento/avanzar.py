#!/usr/bin/env pybricks-micropython


# import
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
# funciones: avanzar hacia delante
def avanzar_adelante(robot: EV3Brick, left_motor: Motor, right_motor: Motor, distancia: float):
    # Inicializar el DriveBase
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
    # Avanzar 20cm
    robot_db.straight(distancia)  # milímetros que avanza
    # pitar
    robot.speaker.beep()


# funciones: avanzar hacia atrás
def avanzar_atras(robot: EV3Brick, left_motor: Motor, right_motor: Motor, distancia: float):
    # Inicializar el DriveBase
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
    # Avanzar 20cm hacia atrás
    robot_db.straight((distancia)*(-1))  # milímetros que avanza
    # pitar
    robot.speaker.beep()
    
