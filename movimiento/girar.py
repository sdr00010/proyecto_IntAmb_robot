from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def girar_derecha(robot: EV3Bricks, grados: float):
    robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
    robot.turn(grados)
    ev3.speaker.beep(frequency=1000, duration=500)

def girar_derecha(robot: EV3Bricks, grados: float):
    robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
    robot.turn((grados) * (-1))
    ev3.speaker.beep(frequency=800, duration=500)