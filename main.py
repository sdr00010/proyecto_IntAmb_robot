# --- !/usr/bin/env pybricks-micropython

# This program requires LEGO EV3 MicroPython v2.0 or higher

# imports
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# imports locales
import os
import sys
import time
from interno import utils
from movimiento import avanzar as avn
from movimiento import girar as gr
from movimiento import accionPaquete as ap

# ---------------------------------------------------------------------------------------------------------------------------------

# Crear el objetos
control = utils.Controlador()
robot = utils.Robot()

# ---------------------------------------------------------------------------------------------------------------------------------

# MAIN DE PRUEBAS
# 1. Generar caminos
# inicial = [31, 26, 21, 16, 11, 12, 13, 18, 23, 28, 33, 34]
# final = [33, 28, 23, 18, 13, 12, 7, 6, 5, 0]

inicial = [23, 18, 13, 12, 7, 6, 5, 0]
final = [5, 6, 7, 12, 13, 14, 9, 4, 3]

# 2. Realizar camino inicial
# print("Se quiere ir de la casilla [30] a la [34], el camino es " + str(inicial))

""" 
    for casilla in inicial:
        print("Robot en " + str(robot.casilla_actual) + " mirando a " + str(robot.orientacion))
        # 1. Comprobar la orientacion
        correcto, deseado = control.comprobar_orientacion(robot.casilla_actual, casilla, robot.orientacion)
        if (not correcto): 
            giro = control.corregir_orientacion(robot.orientacion, deseado)
            print("\tRobot debe orientarse hacia " + str(deseado) + ", girando " + str(giro))
            # Girar robot
            gr.eleccion_giro(robot.drive, giro)
            robot.robot.speaker.beep()
            robot.orientacion = deseado
            print("\t-> Robot en " + str(robot.casilla_actual) + " mirando a " + str(robot.orientacion))
        # else:
            print("\tRobot orientado correctamente")
        # 2. Avanzar a la siquiente casilla
        print("\tRobot avanza 1 casilla, de [" + str(robot.casilla_actual) + "] a [" + str(casilla) + "]")
        avn.adelante_N_casillas(robot.robot, robot.left_motor, robot.right_motor, 1)
        robot.casilla_actual = casilla
    robot.robot.speaker.beep()
    print("Robot recoge el paquete en la casilla " + str(robot.casilla_actual)) 
"""

# ---------------------------------------------------------------------------------------------------------------------------------

""" 
    pala_motor = Motor(Port.B)
    # avanzar una casilla
    avn.adelante_N_casillas(robot.robot, robot.left_motor, robot.right_motor, 1)
    # levantar pala
    pala_motor.run_target(500, 90)
    # pala_motor.stop()
    robot.robot.speaker.beep()
    # avanzar al paquete
    avn.avanzar_paquete_1(robot.robot, robot.left_motor, robot.right_motor)
    robot.robot.speaker.beep()
    # bajar pala
    pala_motor.run_target(500, 0)
    # pala_motor.stop()
    robot.robot.speaker.beep()
    # avanzar al centro
    avn.avanzar_paquete_2(robot.robot, robot.left_motor, robot.right_motor)
    robot.robot.speaker.beep()
    # media vuelta
    robot.drive.turn(180)
    robot.robot.speaker.beep()
    # avanzar a la otra
    avn.avanzar_paquete_1(robot.robot, robot.left_motor, robot.right_motor)
    robot.robot.speaker.beep()
    # levantar pala
    pala_motor.run_target(500, 90)
    pala_motor.stop()
    robot.robot.speaker.beep()
    # retroceder
    avn.atras_paquete(robot.robot, robot.left_motor, robot.right_motor)
    robot.robot.speaker.beep() 
"""

# ---------------------------------------------------------------------------------------------------------------------------------

pedidos = [
    [[31, 26, 21, 16, 11, 12, 13, 18, 23, 28, 33, 34], [33, 28, 23, 18, 13, 12, 7, 6, 5, 0]], 
    [[6, 7, 12, 13, 14, 9, 4, 3], [4, 9, 14, 13, 18, 23, 24]],
    [[18, 13, 12, 11, 16, 21, 26, 31, 30], [31, 26, 21, 16, 11, 12, 13, 18, 23, 28, 33, 34]],
    [[28, 23, 18, 13, 12, 11, 16, 15], [16, 11, 12, 7, 6, 5, 0]]
]

# meter los pedidos en la cola
for p in pedidos:
    control.meter_en_cola(p)
    
index_pedidos = 0


while (True):
    print("Comprobando pedidos...\n")
    pedido = control.siguiente_cola()
    if (pedido):
        print("Realizando pedido...")
        index_pedidos += 1
        for i, camino in enumerate(pedido):
            for j, casilla in enumerate(camino):
                print("\tRobot en " + str(robot.casilla_actual) + " mirando a " + str(robot.orientacion),  end = "\t")
                # 1. Comprobar la orientacion
                correcto, deseado = control.comprobar_orientacion(robot.casilla_actual, casilla, robot.orientacion)
                if (not correcto): 
                    giro = control.corregir_orientacion(robot.orientacion, deseado)
                    print("-> Robot debe orientarse hacia " + str(deseado) + ", girando " + str(giro), end = " ")
                    # Girar robot
                    # gr.eleccion_giro(robot.drive, giro)
                    # robot.robot.speaker.beep()
                    robot.orientacion = deseado
                    print("-> Robot en " + str(robot.casilla_actual) + " mirando a " + str(robot.orientacion))
                else:
                    print("-> Robot orientado correctamente")
                # 2. Avanzar a la siquiente casilla
                # avn.adelante_N_casillas(robot.robot, robot.left_motor, robot.right_motor, 1)
                if (i != 1 or j != len(camino)-1): 
                    robot.casilla_actual = casilla
                else:
                    print("\tRobot entrega el paquete en la casilla [" + str(casilla) + "]")
        # robot.robot.speaker.beep()
        print("\tPedido " + index_pedido + " finalizado\n")
    else:
        time.sleep(2)
            
        