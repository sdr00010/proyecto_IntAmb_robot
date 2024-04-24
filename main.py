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


# Prueba: avanzar hacia delante
#avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 300)

#wait(1000)

# Prueba: avanzar hacia atrás
#avn.avanzar_atras(robot_LEGO, left_motor, right_motor, 300)

# Prueba: girar a la derecha
#gr.girar_derecha(robot_LEGO, left_motor, right_motor, 90)

# Prueba: girar a la izquierda
#gr.girar_izquierda(robot_LEGO, left_motor, right_motor, 90)

# Prueba: giro con el giroscopio
#gr.girar_derecha_gyro(robot_LEGO, right_motor, gyro, 90)

# Prueba: coger paquete
ap.cogerPaquete(robot_LEGO, left_motor, right_motor, pala_motor)
avn.avanzar_adelante(robot_LEGO, left_motor, right_motor, 300)
ap.dejarPaquete(robot_LEGO, left_motor, right_motor, pala_motor)


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
