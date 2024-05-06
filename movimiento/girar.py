#!/usr/bin/env pybricks-micropython

# imports
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

def eleccion_orientacion(destino: str):
    if(destino == "DERECHA"):
        girar_derecha_gyro(robot_LEGO, robot_LEGO.left_motor, robot_LEGO.gyroSensor, 90, robot_LEGO.angle())
    elif(destino == "IZQUIERDA"):
        girar_izquierda_gyro(robot_LEGO, robot_LEGO.right_motor, robot_LEGO.gyroSensor, -90, robot_LEGO.angle())
    elif(destino == "MEDIA-VUELTA"):
        girar_semicirculo_gyro(robot_LEGO, robot_LEGO.left_motor, robot_LEGO.gyroSensor, 180, robot_LEGO.angle())

def girar_derecha_gyro(robot: EV3Brick, left_motor: Motor, gyroSensor: GyroSensor, grados: float, angulo_actual: float):
    angulo_target = angulo_actual + grados
    velocidad = 300
    while angulo_actual < angulo_target:
        left_motor.run(velocidad)
        angulo_actual = gyroSensor.angle()
        print("Angulo actual: ", angulo_actual)
    left_motor.stop(Stop.BRAKE)
    
def girar_izquierda_gyro(robot: EV3Brick, right_motor: Motor, gyroSensor: GyroSensor, grados: float, angulo_actual: float):
    angulo_target = angulo_actual + grados
    velocidad = 300
    while angulo_actual > angulo_target:
        right_motor.run(velocidad)
        angulo_actual = gyroSensor.angle()
        print("Angulo actual: ", angulo_actual)
    right_motor.stop(Stop.BRAKE)

def girar_semicirculo_gyro(robot: EV3Brick, left_motor: Motor, gyroSensor: GyroSensor, grados: float, angulo_actual: float):
    angulo_target = angulo_actual + grados
    velocidad = 300
    while angulo_actual < angulo_target:
        left_motor.run(velocidad)
        angulo_actual = gyroSensor.angle()
        print("Angulo actual: ", angulo_actual)
    left_motor.stop(Stop.BRAKE)