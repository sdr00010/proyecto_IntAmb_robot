#!/usr/bin/env pybricks-micropython

import os
import json
import sys
import time

# imports
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# ----------------------------------------------------------------------------------------------------------------------------------------------------

class Robot:
    # Atributos
    orientacion = ""        # arriba, derecha, abajo, izquierda
    casilla_actual = 0 
    # Robot
    robot = EV3Brick()
    # motores
    left_motor = Motor(Port.A)  # motor rueda izquierda
    right_motor = Motor(Port.D)  # motor rueda izquierda
    pala_motor = Motor(Port.B)  # motor rueda izquierda
    gyroSensor = GyroSensor(Port.S1)
    # drive: movimiento
    drive = DriveBase(  
                        left_motor, 
                        right_motor, 
                        wheel_diameter=55, 
                        axle_track=120
                    )
    # resetear los ángulos
    gyroSensor.reset_angle(0)
    pala_motor.reset_angle(0)

    
    # Constructor
    def __init__(self):
        self.orientacion = "derecha" 
        self.casilla_actual = 30 # abajo a la izquierda


# ----------------------------------------------------------------------------------------------------------------------------------------------------

class Controlador:
    # Configurador: parámetros...
    CONFIG = None
    cola_pedidos = None
    
    def __init__(self):
        # Cargar el archivo de configuración
        with open('interno/config.json', 'r') as f:
            self.CONFIG = json.load(f)
        # Crear la cola de pedidos
            self.cola_pedidos = []
        
    # CAMBIAR QUEUE POR UN LIST
    # Cola de pedidos
    # Funciones de gestión de la cola
    def comprobar_cola_vacia(self):
        return len(self.cola_pedidos) == 0
    def comprobar_cola_no_vacia(self):
        return len(self.cola_pedidos) != 0
    
    def siguiente_cola(self):
        if (self.comprobar_cola_vacia()): return None
        return self.cola_pedidos.pop(0)
    def meter_en_cola(self, elemento: list):
        self.cola_pedidos.append(elemento)
        
    def get_cola(self):
        return self.cola_pedidos.queue
    
    # Funciones del robot
    def comprobar_orientacion(self, c_inicial, c_final, orientacion):
        orient = c_inicial - c_final
        relacion = self.CONFIG["parametros"]["relaciones"][str(orient)] # -1: derecha, 1: izquierda, 5: arriba, -5: abajo
        if (orientacion == relacion): 
            return True, relacion
        else: 
            return False, relacion
    def corregir_orientacion(self, actual, deseada):
        dir_actual = self.CONFIG["parametros"]["direcciones"][actual]   # arriba = 1
        dir_deseada = self.CONFIG["parametros"]["direcciones"][deseada] # derecha = 2
        # resta
        resultado = dir_deseada - dir_actual # derecha (2) - arriba (1) = 1
        # devolver el giro adecuado
        if (resultado == 1 or resultado == -3): return "DERECHA"
        if (abs(resultado) == 2): return "MEDIA-VUELTA"
        if (resultado == -1 or resultado == 3): return "IZQUIERDA"
        
        

# ----------------------------------------------------------------------------------------------------------------------------------------------------
      


