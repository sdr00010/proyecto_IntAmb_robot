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

# ----------------------------------------------------------------------------------------------------------------------------------------------------

class ControladorGiro:
    robot = None
    controlador = None
    # constructor
    def __init__(self, robot: Robot):
        self.robot = robot
        self.controlador = Controlador()
    
    # métodos privados
    
    def __girar_derecha(self):
        # ángulos: actual y destino
        actual = self.robot.gyroSensor.angle()
        destino = self.robot.gyroSensor.angle() + self.controlador.CONFIG["parametros"]["giros"]["derecha"]
        # girar
        self.robot.drive.turn(self.controlador.CONFIG["parametros"]["giros"]["derecha"])
        # corregir el angulo: giroscopio
        self.robot.drive.stop()
        while self.robot.gyroSensor.angle() < destino:
            self.robot.left_motor.run(100)
    
    def __girar_izquierda(self):
        # ángulos: actual y destino
        actual = self.robot.gyroSensor.angle()
        destino = self.robot.gyroSensor.angle() + self.controlador.CONFIG["parametros"]["giros"]["izquierda"]
        # girar
        self.robot.drive.turn(self.controlador.CONFIG["parametros"]["giros"]["izquierda"])
        # corregir el angulo: giroscopio
        self.robot.drive.stop()
        while self.robot.gyroSensor.angle() > destino:
            self.robot.right_motor.run(100)
    
    def __girar_media_vuelta(self):
        # ángulos: actual y destino
        actual = self.robot.gyroSensor.angle()
        destino = self.robot.gyroSensor.angle() + self.controlador.CONFIG["parametros"]["giros"]["vuelta"]
        # girar
        self.robot.drive.turn(self.controlador.CONFIG["parametros"]["giros"]["vuelta"])
        # corregir el angulo: giroscopio
        self.robot.drive.stop()
        while self.robot.gyroSensor.angle() < destino:
            self.robot.left_motor.run(140)
    
    # métodos públicos
    
    def eleccion_giro(self, destino: str):
        if(destino == "DERECHA"):
            self.__girar_derecha()
        elif(destino == "IZQUIERDA"):
            self.__girar_izquierda()
        elif(destino == "MEDIA-VUELTA"):
            self.__girar_media_vuelta()


# ----------------------------------------------------------------------------------------------------------------------------------------------------

def girar_derecha(robot: Robot):
    actual = robot.gyroSensor.angle()
    destino = robot.gyroSensor.angle() + (controlador.CONFIG["parametros"]["giro"] - 2)
    print("angulo actual --> ", actual, "destino --> ", destino)
    robot.drive.turn((controlador.CONFIG["parametros"]["giro"] - 1))
    # corregir el angulo
    robot.drive.stop()
    while robot.gyroSensor.angle() < destino:
        robot.left_motor.run(150)
        # print(robot.colorSensor.color())
        # if(robot.colorSensor.color() != Color.GREEN):
        #     print(robot.colorSensor.color())
    print("conseguido -->", robot.gyroSensor.angle())
    

def girar_izquierda(robot: Robot):
    actual = robot.gyroSensor.angle()
    destino = robot.gyroSensor.angle() + ((controlador.CONFIG["parametros"]["giro"]-1) * (-1))
    print("angulo actual --> ", actual, "destino --> ", destino)
    robot.drive.turn((controlador.CONFIG["parametros"]["giro"]-1) * (-1))
    # corregir el angulo
    robot.drive.stop()
    while robot.gyroSensor.angle() > destino:
        robot.right_motor.run(160)
        # print(robot.colorSensor.color())
        # if(robot.colorSensor.color() != Color.GREEN):
        #     print(robot.colorSensor.color())
    print("conseguido -->", robot.gyroSensor.angle())
    
def girar_media_vuelta(robot: Robot):
    actual = robot.gyroSensor.angle()
    destino = robot.gyroSensor.angle() + (controlador.CONFIG["parametros"]["giro"])*2
    print("angulo actual --> ", actual, "destino --> ", destino)
    robot.drive.turn((controlador.CONFIG["parametros"]["giro"])*2)
    # corregir el angulo
    robot.drive.stop()
    while robot.gyroSensor.angle() < destino:
        robot.left_motor.run(160)
        # print(robot.colorSensor.color())
        # if(robot.colorSensor.color() != Color.GREEN):
        #     print(robot.colorSensor.color())
    print("conseguido -->", robot.gyroSensor.angle())
    
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