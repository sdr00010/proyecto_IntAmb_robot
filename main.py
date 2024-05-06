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
from movimiento import avanzar as avn
from movimiento import girar as gr
from movimiento import accionPaquete as ap

# ---------------------------------------------------------------------------------------------------------------------------------

# Crear el objeto del robot.
robot_LEGO = EV3Brick()
left_motor = Motor(Port.A)  # motor rueda izquierda
right_motor = Motor(Port.D) # motor rueda derecha
pala_motor = Motor(Port.B)
gyro = GyroSensor(Port.S1) # giroscopio

os.system('hostname > /dev/shm/hostname.txt')
file = open('/dev/shm/hostname.txt', 'r')
client_id = file.readline().rstrip('\n')
file.close()
os.system('rm /dev/shm/hostname.txt')

mqtt_broker = '127.0.0.1'
mqtt_password = 'swifi4680'
mqtt_port = 1883

print(client_id)

mqtt_client = MQTTClient(client_id, mqtt_broker, port=mqtt_port, password=mqtt_password)

print("mqtt client creado")

MQTT_Topic_Mapa = 'mapa'
MQTT_Topic_Pedido = 'pedido'
MQTT_Topic_Posicion = 'posicion'
MQTT_Topic_Finalizacion = 'finalizacion'

def on_connect():
    print("Conectado al Broker MQTT")
    mqtt_client.subscribe(MQTT_Topic_Mapa)
    mqtt_client.subscribe(MQTT_Topic_Pedido)
    mqtt_client.subscribe(MQTT_Topic_Posicion)
    mqtt_client.subscribe(MQTT_Topic_Finalizacion)

def on_message(topic, msg):
    print("Received message: " + msg.decode() + " from topic: " + topic)
    # Add the received message to the queue
    #movement_queue.put(msg.decode())

def on_disconnect():
    print("Desconectado del Broker MQTT")


def execute_movement():
    while not movement_queue.empty():  # comprobar la cola de pedidos y moverse
        command = movement_queue.get()
        #print(f"Executing command: {command}")
        if command == "forward":
            left_motor.run_timed(time_sp=2000, speed_sp=500)
            right_motor.run_timed(time_sp=2000, speed_sp=500)
        elif command == "stop":
            left_motor.stop()
            right_motor.stop()
        else:
            print("Unknown command")
        # Simulate a delay to represent the completion of a movement series
        time.sleep(5)

# Connect to MQTT Broker
mqtt_client.set_callback(on_message)
print("conectando")
mqtt_client.connect()
print("conectado")
on_connect()

try:
    while True:
        #execute_movement()
        # Optionally, publish the robot's status every few seconds
        status_msg = "Robot Status: conectadoo"
        mqtt_client.publish(MQTT_Topic_Posicion, status_msg.encode())
        time.sleep(5)
except KeyboardInterrupt:
    on_disconnect()
    mqtt_client.disconnect()


# Prueba: avanzar hacia delante
#avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 300)

#wait(1000)

# Prueba: avanzar hacia atrás
#avn.avanzar_atras(robot_LEGO, left_motor, right_motor, 300)

# Prueba: girar a la derecha
#gr.girar_derecha(robot_LEGO, left_motor, right_motor, 180)

# Prueba: girar a la izquierda
#gr.girar_izquierda(robot_LEGO, left_motor, right_motor, 90)

# Prueba: giro con el giroscopio
#global float angulo_actual = 0
#gr.girar_derecha_gyro(robot_LEGO, left_motor, gyro, 180, gyro.angle())

#gr.girar_izquierda_gyro(robot_LEGO, right_motor, gyro, -180, gyro.angle())

# Prueba: coger paquete
#ap.cogerPaquete(robot_LEGO, left_motor, right_motor, pala_motor)
#avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 300)
#ap.dejarPaquete(robot_LEGO, left_motor, right_motor, pala_motor)


""" 
# 1. Make a sound.
ev3.speaker.beep()

# 2. Initialize a motor at port B.
test_motor = Motor(Port.A)
# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
test_motor.run_target(500, 90)

# 3. Play another beep sound.
ev3.speaker.beep(frequency=1000, duration=500) 
"""
