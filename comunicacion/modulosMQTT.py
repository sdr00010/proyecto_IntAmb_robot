
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

MQTT_Topic_Mapa = 'something/mapa'
MQTT_Topic_Pedido = 'something/pedido'
MQTT_Topic_Posicion = 'something/posicion'
MQTT_Topic_Finalizacion = 'something/finalizacion'

def getmessages(topic, msg):
    if topic == MQTT_Topic_Mapa.encode():
        #brick.display.text(str(msg.decode()))
        #procesarmapa()
    elif topic == MQTT_Topic_Pedido.encode():
        #brick.display.text(str(msg.decode()))
        #motor.run_target(180, int(msg.decode()))
        #procesarpedido

getmessage()

client = MQTTClient(MQTT_ClientID, MQTT_Broker)
client.connect()

def publicarPosicion(client, topic):
    client.publish(MQTT_Topic_Posicion, MQTT_ClientID + ' posicion')

def publicarFinalizacion(topic):
    client.publish(MQTT_Topic_Finalizacion, MQTT_ClientID + ' finalizado')

