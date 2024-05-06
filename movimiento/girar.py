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

def girar_derecha(robot_db: DriveBase):
    robot_db.turn(controlador.CONFIG["parametros"]["giro"])
    #robot.speaker.beep(frequency=1000, duration=500)

def girar_izquierda(robot_db: DriveBase):
    robot_db.turn((controlador.CONFIG["parametros"]["giro"] + 1) * (-1))
    
    #robot.speaker.beep(frequency=800, duration=500)
    
def girar_media_vuelta(robot_db: DriveBase):
    robot_db.turn(180)
    
def eleccion_giro(robot_drive: DriveBase, destino: str):
    if(destino == "DERECHA"):
        girar_derecha(robot_drive)
    elif(destino == "IZQUIERDA"):
        girar_izquierda(robot_drive)
    elif(destino == "MEDIA-VUELTA"):
        girar_media_vuelta(robot_drive)


def eleccion_orientacion(robot_LEGO: Robot, destino: str):
    if(destino == "DERECHA"):
        girar_derecha_gyro(robot_LEGO.robot, robot_LEGO.left_motor, robot_LEGO.gyroSensor, 90, robot_LEGO.gyroSensor.angle())
    elif(destino == "IZQUIERDA"):
        girar_izquierda_gyro(robot_LEGO.robot, robot_LEGO.right_motor, robot_LEGO.gyroSensor, -90, robot_LEGO.gyroSensor.angle())
    elif(destino == "MEDIA-VUELTA"):
        girar_semicirculo_gyro(robot_LEGO.robot, robot_LEGO.left_motor, robot_LEGO.gyroSensor, 180, robot_LEGO.gyroSensor.angle())

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