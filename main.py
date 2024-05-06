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
from umqtt.simple import MQTTClient

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
            
        