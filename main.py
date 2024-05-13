#!/usr/bin/env pybricks-micropython

# This program requires LEGO EV3 MicroPython v2.0 or higher

# imports
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from umqtt.simple import MQTTClient

# imports locales
import os
import sys
import time

# módulos
from interno.utils import Controlador, Robot
from interno.comunicacionMQTT import ControladorMQTT
from movimiento.avanzar import ControladorMovimiento
from movimiento.girar import ControladorGiro

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Crear el objetos
control = utils.Controlador()
robot = utils.Robot(
    control.CONFIG["parametros"]["robot"]["wheel_diameter"],
    control.CONFIG["parametros"]["robot"]["axle_track"],
    control.CONFIG["parametros"]["robot"]["straight_speed"],
    control.CONFIG["parametros"]["robot"]["straight_acceleration"],
    control.CONFIG["parametros"]["robot"]["turn_rate"],
    control.CONFIG["parametros"]["robot"]["turn_acceleration"]
)
ControladorMovimiento = ControladorMovimiento(robot)
ControladorGiro = ControladorGiro(robot)
ControladorMQTT = ControladorMQTT(control)

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

"""
    # Configuracion de la comunicación mqtt

    os.system('hostname > /dev/shm/hostname.txt')
    file = open('/dev/shm/hostname.txt', 'r')
    client_id = file.readline().rstrip('\n')
    file.close()
    os.system('rm /dev/shm/hostname.txt')

    mqtt_broker = '192.168.171.45' # cambiar cada dia
    mqtt_password = 'swifi4680'
    mqtt_port = 1883

    mqtt_client = MQTTClient(client_id, mqtt_broker, port=mqtt_port, password=mqtt_password)

    MQTT_Topic_Mapa = 'map'
    MQTT_Topic_Pedido = 'equipoD/pedido'
    MQTT_Topic_Posicion = 'equipoD/posicion'
    MQTT_Topic_Finalizacion = 'equipoD/finalizacion'


    # subcribirse a los topics
    def on_connect():
        print("Conectado al Broker MQTT")
        mqtt_client.subscribe(MQTT_Topic_Pedido)
        mqtt_client.check_msg()

    # procesar el mensaje de pedido
    def procesar_pedido(pedido_decoded):
        caminos = pedido_decoded.strip('"').split(";")
        camino_inicio = [int(x) for x in (caminos[0].split(","))]
        camino_final = [int(x) for x in (caminos[1].split(","))]
        return [camino_inicio, camino_final]

    # llega un mensaje
    def on_message(topic, msg):
        print("Received message: " + str(msg.decode()) + " from topic: " + str(topic))
        # comprobar topic
        if (MQTT_Topic_Pedido in str(topic)):
            pedido_decoded = str(msg.decode())
            pedido = procesar_pedido(pedido_decoded)
            control.meter_en_cola(pedido)

    # desconcexion del broker
    def on_disconnect():
        print("Desconectado del Broker MQTT")
        
    # conexión
    mqtt_client.set_callback(on_message)
    mqtt_client.connect()
    on_connect()
    robot.robot.speaker.beep()
"""

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# control.meter_en_cola([[31, 26, 21, 16, 11, 12, 13, 18, 23, 28, 27], [28, 23, 18, 13, 12, 7, 6, 1]])
# control.meter_en_cola([[7, 12, 11, 16, 15], [16, 11, 12, 13, 18, 23, 28, 33, 34]])

# MAIN

MQTT_Topic_Mapa = 'map'
MQTT_Topic_Pedido = 'equipoD/pedido'
MQTT_Topic_Posicion = 'equipoD/posicion'
MQTT_Topic_Finalizacion = 'equipoD/finalizacion'

# conexión
ControladorMQTT.client.connect()
ControladorMQTT.on_connect()
robot.robot.speaker.beep()

# tarea: reparto de pedidos
try:
    while True:
        # Comprobar la cola de pedidos
        pedido = control.siguiente_cola()
        if ( pedido ):
            # 1 pedido = 2 listas de casillas
            for i, camino in enumerate(pedido):
                for j, casilla in enumerate(camino):
                    # 1. Comprobar la orientacion
                    correcto, deseado = control.comprobar_orientacion(robot.casilla_actual, casilla, robot.orientacion)
                    if (not correcto): 
                        giro = control.corregir_orientacion(robot.orientacion, deseado)
                        # Girar robot
                        ControladorGiro.eleccion_giro(robot, giro)
                        robot.orientacion = deseado
                    # 2. Avanzar a la siquiente casilla
                    if ( not (i == 1 and j == len(camino)-1)): # no es el destino
                        if ( (i == 0 and j == len(camino)-1) ): # coger el paquete
                            ControladorMovimiento.recoger_paquete()
                        else: # continuar con el camino
                            ControladorMovimiento.adelante_casillas(1)
                        robot.casilla_actual = casilla
                        # publicar posicion: odometría
                        ControladorMQTT.publicar_mensaje(MQTT_Topic_Posicion, str(robot.casilla_actual))
            # Entregar paquete
            ControladorMovimiento.entregar_paquete()
            robot.robot.speaker.beep()
            # publicar finalización
            ControladorMQTT.publicar_mensaje(MQTT_Topic_Finalizacion, "finalizado")
        else:
            # Esperar a un nuevo pedido
            robot.drive.stop()
            ControladorMQTT.client.check_msg()
            time.sleep(2)
            
except KeyboardInterrupt:
    ControladorMQTT.on_disconnect()
    ControladorMQTT.client.disconnect()


# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

""" 
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

"""
            
        