#!/usr/bin/env pybricks-micropython

# imports
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from interno.utils import Robot

from interno.utils import Controlador
controlador = Controlador()

def girar_derecha(robot: Robot):
    actual = robot.gyroSensor.angle()
    destino = robot.gyroSensor.angle() + (controlador.CONFIG["parametros"]["giro"] + 0)
    robot.drive.turn((controlador.CONFIG["parametros"]["giro"] + 0))
    # corregir el angulo
    robot.drive.stop()
    while robot.gyroSensor.angle() < destino:
        robot.left_motor.run(250)
        print(robot.colorSensor.color())
        if(robot.colorSensor.color() != Color.GREEN):
            print(robot.colorSensor.color())
    

def girar_izquierda(robot: Robot):
    actual = robot.gyroSensor.angle()
    destino = robot.gyroSensor.angle() + ((controlador.CONFIG["parametros"]["giro"]-1) * (-1))
    robot.drive.turn((controlador.CONFIG["parametros"]["giro"]-1) * (-1))
    # corregir el angulo
    robot.drive.stop()
    while robot.gyroSensor.angle() > destino:
        robot.right_motor.run(200)
        print(robot.colorSensor.color())
        if(robot.colorSensor.color() != Color.GREEN):
            print(robot.colorSensor.color())
    
def girar_media_vuelta(robot: Robot):
    actual = robot.gyroSensor.angle()
    destino = robot.gyroSensor.angle() + (controlador.CONFIG["parametros"]["giro"])*2
    robot.drive.turn(controlador.CONFIG["parametros"]["giro"]*2)
    # corregir el angulo
    robot.drive.stop()
    while robot.gyroSensor.angle() < destino:
        robot.left_motor.run(250)
        print(robot.colorSensor.color())
        if(robot.colorSensor.color() != Color.GREEN):
            print(robot.colorSensor.color())
    
def eleccion_giro(robot: Robot, destino: str):
    if(destino == "DERECHA"):
        girar_derecha(robot)
    elif(destino == "IZQUIERDA"):
        girar_izquierda(robot)
    elif(destino == "MEDIA-VUELTA"):
        girar_media_vuelta(robot)


def girar_derecha_gyro(robot: EV3Brick, left_motor: Motor, gyroSensor: GyroSensor, grados: float, angulo_actual: float):
    angulo_target = angulo_actual + grados
    velocidad = 300
    while angulo_actual < angulo_target:
        left_motor.run(velocidad)
        angulo_actual = gyroSensor.angle()
        # print("Angulo actual: ", angulo_actual)
    # left_motor.stop(Stop.BRAKE)
    
def girar_izquierda_gyro(robot: EV3Brick, right_motor: Motor, gyroSensor: GyroSensor, grados: float, angulo_actual: float):
    angulo_target = angulo_actual + grados
    velocidad = 300
    while angulo_actual > angulo_target:
        right_motor.run(velocidad)
        angulo_actual = gyroSensor.angle()
        # print("Angulo actual: ", angulo_actual)
    # right_motor.stop(Stop.BRAKE)

def girar_semicirculo_gyro(robot: EV3Brick, left_motor: Motor, gyroSensor: GyroSensor, grados: float, angulo_actual: float):
    angulo_target = angulo_actual + grados
    velocidad = 300
    while angulo_actual < angulo_target:
        left_motor.run(velocidad)
        angulo_actual = gyroSensor.angle()
        # print("Angulo actual: ", angulo_actual)
    # left_motor.stop(Stop.BRAKE)