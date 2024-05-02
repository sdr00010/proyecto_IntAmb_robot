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

class Celda:
    id = 0
    tipo = ""
    nombre = ""
    arriba = False
    derecha = False
    abajo = False
    izquierda = False
    def __init__(self, id, tipo, datos):
        self.id = id
        self.tipo = tipo
        self.nombre = datos["nombre"]
        self.arriba = datos["arriba"]
        self.derecha = datos["derecha"]
        self.abajo = datos["abajo"]
        self.izquierda = datos["izquierda"]
    # def __str__(self):
    #     return f"[{self.id}] {self.nombre} -> arriba-{self.arriba} ; derecha-{self.derecha} ; abajo-{self.abajo} ; izquierda-{self.izquierda}"

class Casilla:
    id = ""
    row = 0
    col = 0
    def __init__(self, id, row, col):
        self.id = id
        self.row = row  
        self.col = col

class Pedido:
    id = ""
    inicio = Casilla("", 0, 0)
    destino = Casilla("", 0, 0)
    def __init__(self, id, inicio, destino):
        self.id = id
        self.inicio = inicio  
        self.destino = destino

class Robot:
    # Atributos
    name = ""
    estado = ""             # parado, trabajando
    orientacion = ""        # arriba, derecha, abajo, izquierda
    casilla_actual = 0 
    # Robot
    robot = EV3Brick()
    left_motor = Motor(Port.A)  # motor rueda izquierda
    right_motor = Motor(Port.D) # motor rueda derecha
    drive = None
    # Constructor
    def __init__(self):
        self.name = "robotito"
        self.estado = "parado"         
        self.orientacion = "derecha" 
        self.casilla_actual = 30 # abajo a la izquierda
        self.drive = DriveBase(  
                                self.left_motor, 
                                self.right_motor, 
                                wheel_diameter=55, 
                                axle_track=120
                            )

# ----------------------------------------------------------------------------------------------------------------------------------------------------

class Controlador:
    # Configurador: parámetros...
    CONFIG = None
    
    def __init__(self):
        # Cargar el archivo de configuración
        with open('interno/config.json', 'r') as f:
            self.CONFIG = json.load(f)
        # Crear la cola de pedidos
            # self.__cola_pedidos = Queue()
        # Cargar el self.mapa
            # mapa_sin_procesar = self._cargar()
        # Procesar el self.mapa
            # self.__procesar_mapa()
        
    # CAMBIAR QUEUE POR UN LIST
    """ # Cola de pedidos
    # Funciones de gestión de la cola
        def comprobar_cola_vacia(self):
            return self.__cola_pedidos.empty()
        def comprobar_cola_no_vacia(self):
            return (not self.__cola_pedidos.empty())
        def siguiente_cola(self):
            if (self.__cola_pedidos.empty()): return None
            return __cola_pedidos.get_nowait()
        def meter_en_cola(self, elemento: str):
            self.__cola_pedidos.put_nowait(elemento)
        def get_cola(self):
            return self.__cola_pedidos.queue """
    
    # Funciones del robot
    def comprobar_orientacion(self, c_inicial, c_final, orientacion):
        relacion = self.CONFIG["parametros"]["relaciones"][str(c_inicial - c_final)] # -1: derecha, 1: izquierda, 5: arriba, -5: abajo
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
        if (resultado == 1 or resultado == -3): return 90
        if (abs(resultado) == 2): return 180
        if (resultado == -1 or resultado == 3): return -90
        
        

# ----------------------------------------------------------------------------------------------------------------------------------------------------
      

