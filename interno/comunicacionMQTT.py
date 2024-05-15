#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


from umqtt.robust import MQTTClient
import time
import os
from interno.utils import Controlador

# ----------------------------------------------------------------------------------------------------------------------------------------------------

class ControladorMQTT:
    
    client = None
    controlador = None

    def __init__(self, controlador):
        
        os.system('hostname > /dev/shm/hostname.txt')
        file = open('/dev/shm/hostname.txt', 'r')
        client_id = file.readline().rstrip('\n')
        file.close()
        os.system('rm /dev/shm/hostname.txt')
        
        self.controlador = controlador
        self.client = MQTTClient(client_id, 
                                 self.controlador.CONFIG["parametros"]["mqtt"]["brokerIP"], 
                                 port=self.controlador.CONFIG["parametros"]["mqtt"]["port"], 
                                 password=self.controlador.CONFIG["parametros"]["mqtt"]["password"]
                                )
        
        self.client.set_callback(self.on_message)
        
    # métodos privados
        
    # procesar el mensaje de pedido
    def __procesar_pedido(self, pedido_decoded):
        caminos = pedido_decoded.strip('"').split(";")
        camino_inicio = [int(x) for x in (caminos[0].split(","))]
        camino_final = [int(x) for x in (caminos[1].split(","))]
        return [camino_inicio, camino_final]

    # métodos públicos

    # llega un mensaje
    def on_message(self, topic, msg):
        pedido_decoded = str(msg.decode())
        pedido = self.__procesar_pedido(pedido_decoded)
        self.controlador.meter_en_cola(pedido)

    def on_disconnect(self):
        print("Desconectado del Broker MQTT")
        
    def publicar_mensaje(self, topic, mensaje):
        self.client.publish(topic, mensaje.encode())
        
        

