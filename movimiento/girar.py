from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def girar_derecha(robot: EV3Brick, left_motor: Motor, right_motor: Motor, grados: float):
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=120)
    robot_db.turn(grados)
    #robot.speaker.beep(frequency=1000, duration=500)

def girar_izquierda(robot: EV3Brick, left_motor: Motor, right_motor: Motor, grados: float):
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=120)
    robot_db.turn((grados) * (-1))
    #robot.speaker.beep(frequency=800, duration=500)

def girar_derecha_gyro(robot: EV3Brick, right_motor: Motor, gyroSensor: GyroSensor, grados: float):
    print("Velocidad de giro: " gyroSensor.speed())
    right_motor.dc(gyroSensor.angle(grados))
    