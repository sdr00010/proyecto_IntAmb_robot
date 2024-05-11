#!/usr/bin/env pybricks-micropython

# import
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from interno.utils import Robot

# imports locales
from interno.utils import Controlador

# ----------------------------------------------------------------------------------------------------------------------------------------------------

class ControladorMovimiento:
    robot = None
    controlador = None
    # constructor
    def __init__(self, robot: Robot):
        self.robot = robot
        self.controlador = Controlador()
    # métodos privados
    def __avanzar_adelante(self, distancia: float):
        self.robot.drive.robot_db.straight(distancia)
    def __avanzar_atras(self, distancia: float):
        self.robot.drive.robot_db.straight(distancia*(-1))
    # métodos públicos
    # 1. Avanzar N casillas hacia delante
    def avanzar_casillas(self, n: int):
        self.__avanzar_adelante( n*(self.controlador.CONFIG["parametros"]["medidas"]["casilla"]) )
    # 2. Avanzar N casillas hacia atrás
    def retroceder_casillas(self, n: int):
        self.__avanzar_atras( n*(self.controlador.CONFIG["parametros"]["medidas"]["casilla"]) )
    # 3. Avanzar al paquete
    def avanzar_al_paquete(self):
        self.__avanzar_adelante(self.controlador.CONFIG["parametros"]["medidas"]["al_paquete"]) # medida entre el (centro anterior) hasta la (posicion de coger el paquete)
    # 4. Recolocarse en la casilla del paquete
    def avanzar_recolocar_paquete(self):
        self.__avanzar_adelante(self.controlador.CONFIG["parametros"]["medidas"]["recolocar_paquete"]) # medida entre (posicion de coger el paquete) y (centro de la casilla siguiente)
    # 5. Retroceder tras entregar el paquete
    def retroceder_paquete(self):
        self.__avanzar_atras(self.controlador.CONFIG["parametros"]["medidas"]["al_paquete"])
        
    def recoger_paquete(self):
        self.avanzar_al_paquete() # avanzar delante del paquete
        wait(500) # esperar
        self.robot.pala_motor.run_target(self.controlador.CONFIG["parametros"]["pala"]["velocidad"], self.controlador.CONFIG["parametros"]["pala"]["bajar"]) # bajar la pala
        wait(800) # esperar
        self.avanzar_recolocar_paquete() # avanzar al medio de la casilla
        self.robot.pala_motor.stop()

    def entregar_paquete(self):
        self.avanzar_al_paquete() # avanzar delante del paquete
        self.robot.pala_motor.run_target(self.controlador.CONFIG["parametros"]["pala"]["velocidad"], self.controlador.CONFIG["parametros"]["pala"]["subir"])  # subir pala
        wait(800) # esperar
        self.retroceder_paquete() # retroceder a la anterior
        self.robot.pala_motor.stop()
    
    
# funciones: avanzar hacia delante
def avanzar_adelante(robot: EV3Brick, left_motor: Motor, right_motor: Motor, distancia: float):
    # Inicializar el DriveBase
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=controlador.CONFIG["parametros"]["wheel_diameter"], axle_track=controlador.CONFIG["parametros"]["axle_track"])
    # Avanzar 20cm
    robot_db.straight(distancia)  # milímetros que avanza
    # pitar
    # robot.speaker.beep()

# funciones: avanzar hacia atrás
def avanzar_atras(robot: EV3Brick, left_motor: Motor, right_motor: Motor, distancia: float):
    # Inicializar el DriveBase
    robot_db = DriveBase(left_motor, right_motor, wheel_diameter=controlador.CONFIG["parametros"]["wheel_diameter"], axle_track=controlador.CONFIG["parametros"]["axle_track"])
    # Avanzar 20cm hacia atrás
    robot_db.straight((distancia)*(-1))  # milímetros que retrocede
    # pitar
    # robot.speaker.beep()

# funciones: avanzar N casillas hacia delante
def adelante_N_casillas(robot: EV3Brick, left_motor: Motor, right_motor: Motor, n: int):
    avanzar_adelante(robot, left_motor, right_motor, n*(controlador.CONFIG["parametros"]["medida_casilla"]))

# funciones: avanzar N casillas hacia atrás
def atras_N_casillas(robot: EV3Brick, left_motor: Motor, right_motor: Motor, n: int):
    avanzar_atras(robot, left_motor, right_motor, n*(controlador.CONFIG["parametros"]["medida_casilla"]))

# funciones para recoger el paquete
def avanzar_paquete_1(robot: EV3Brick, left_motor: Motor, right_motor: Motor):
    avanzar_adelante(robot, left_motor, right_motor, 170) # medida entre el (centro anterior) hasta la (posicion de coger el paquete)
    
def avanzar_paquete_2(robot: EV3Brick, left_motor: Motor, right_motor: Motor):
    avanzar_adelante(robot, left_motor, right_motor, 1) # medida entre (posicion de coger el paquete) y (centro de la casilla siguiente)
    
def atras_paquete(robot: EV3Brick, left_motor: Motor, right_motor: Motor):
    avanzar_atras(robot, left_motor, right_motor, 150) # medida entre el (centro anterior) hasta la (posicion de coger el paquete)
    
    