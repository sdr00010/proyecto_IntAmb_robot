#!/usr/bin/env pybricks-micropython


# import
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from movimiento import avanzar as avn

def cogerPaquete(robot_LEGO: EV3Brick, left_motor: Motor, right_motor: Motor, pala_motor: Motor):
    pala_motor.reset_angle(-50)
    avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 150)
    pala_motor.run_target(500, 90)
    wait(1000)
    pala_motor.run_target(500, -50)
    pala_motor.stop()
    avn.avanzar_atras(robot_LEGO, left_motor, right_motor, 150)


def dejarPaquete(robot_LEGO: EV3Brick, left_motor: Motor, right_motor: Motor, pala_motor: Motor):
    avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 150)
    pala_motor.reset_angle(-50)
    pala_motor.run_target(500, 90)
    wait(1000)
    avn.avanzar_atras(robot_LEGO, left_motor, right_motor, 150)
    pala_motor.run_target(500, -50)
    pala_motor.stop()
