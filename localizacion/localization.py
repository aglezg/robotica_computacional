#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


# Declaración de funciones
def distancia(a, b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2], b[:2]))

def angulo_rel(pose, p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1] - pose[1], p[0] - pose[0]) - pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def mostrar(objetivos,ideal,trayectoria):
  # Mostrar objetivos y trayectoria:
  plt.ion() # modo interactivo
  # Fijar los bordes del gr�fico
  objT   = np.array(objetivos).T.tolist()
  trayT  = np.array(trayectoria).T.tolist()
  ideT   = np.array(ideal).T.tolist()
  bordes = [min(trayT[0]+objT[0]+ideT[0]),max(trayT[0]+objT[0]+ideT[0]),
            min(trayT[1]+objT[1]+ideT[1]),max(trayT[1]+objT[1]+ideT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])*.75
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar objetivos y trayectoria
  idealT = np.array(ideal).T.tolist()
  plt.plot(idealT[0],idealT[1],'-g')
  plt.plot(trayectoria[0][0],trayectoria[0][1],'or')
  r = radio * .1
  for p in trayectoria:
    plt.plot([p[0],p[0]+r*cos(p[2])],[p[1],p[1]+r*sin(p[2])],'-r')
    #plt.plot(p[0],p[1],'or')
  objT   = np.array(objetivos).T.tolist()
  plt.plot(objT[0],objT[1],'-.o')
  plt.show()
  raw_input()
  plt.clf()

def localizacion(balizas, real, ideal, centro, radio, mostrar=0):

  # Buscar la localización más probable del robot, a partir de su sistema
  # sensorial, dentro de una región cuadrada de centro "centro" y lado "2*radio".
  imagen = []
  inc = 0.05                            # Incremento
  err = float('inf')                    # Error de medida
  idealPose = [-1, -1, -1]              # Posición ideal
  markerDistances = real.sense(balizas) # Distancia a balizas
  

  for i in np.arange(-radio, radio, inc):
    imagen.append([])
    for j in np.arange(-radio, radio, inc):
      # Posicion ideal de acuerdo a la iteración
      ideal.set(centro[0] + i, centro[1] + j, markerDistances[-1])
      # Obtención del nuevo error
      actualErr = ideal.measurement_prob(markerDistances, balizas)
      # Guardamos el error en la matriz de imagen
      imagen[-1].append(actualErr)
      # Comprobamos errores
      if actualErr < err:
        err = actualErr
        idealPose = ideal.pose()

  # Pose final, con el menor error asignado
  ideal.set(*idealPose)

  if mostrar:
    plt.ion()  # modo interactivo
    plt.xlim(centro[0] - radio, centro[0] + radio)
    plt.ylim(centro[1] - radio, centro[1] + radio)
    imagen.reverse()
    plt.imshow(
      imagen,
      extent=[
        centro[0] - radio,
        centro[0] + radio,
        centro[1] - radio,
        centro[1] + radio,
      ],
    )
    balT = np.array(balizas).T.tolist()
    #plt.plot(idealPose[0], idealPose[1], "D", c="#ffffff", ms=10, mew=2)
    plt.plot(balT[0], balT[1], "or", ms=10)
    plt.plot(ideal.x, ideal.y, "D", c="#ff00ff", ms=10, mew=2)
    plt.plot(real.x, real.y, "D", c="#00ff00", ms=10, mew=2)
    plt.show()
    raw_input()
    plt.clf()

# Definición del robot:
P_INICIAL = [0., 4., 0.]   # Pose inicial (posición y orientación)
V_LINEAL = .7              # Velocidad lineal    (m/s)
V_ANGULAR = 140.           # Velocidad angular   (rad/s)
FPS = 10.                  # Resolución temporal (fps)
HOLONOMICO = 1 # 0 = triciclo
GIROPARADO = 0
LONGITUD = 0.2

# Definición de trayectorias:
trayectorias = [
    [[1, 3]],
    [[0, 2], [4, 2]],
    [[2, 4], [4, 0], [0, 0]],
    [[2, 4], [2, 0], [0, 2], [4, 2]],
    [[2 + 2 * sin(0.8 * pi * i), 2 + 2 * cos(0.8 * pi * i)] for i in range(5)],
]

# Definición de los puntos objetivo:
if len(sys.argv) < 2 or int(sys.argv[1]) < 0 or int(sys.argv[1]) >= len(trayectorias):
    sys.exit(sys.argv[0] + " <Índice entre 0 y " + str(len(trayectorias) - 1) + ">")
objetivos = trayectorias[int(sys.argv[1])]

# Definición de constantes:
EPSILON = .1                     # Umbral de distancia
V = V_LINEAL / FPS               # Metros por fotograma
W = V_ANGULAR * pi / (180 * FPS) # Radianes por fotograma

ideal = robot()
ideal.set_noise(0, 0, 0.1) # Ruido lineal / radial / de sensado
ideal.set(*P_INICIAL)

real = robot()
real.set_noise(0.01, 0.01, 0.1)  # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

random.seed(0)
tray_ideal = [ideal.pose()] # Trayectoria percibida
tray_real = [real.pose()]   # Trayectoria seguida

tiempo = 0.0
espacio = 0.0
random.seed(0)
#random.seed(datetime.now())

# Localización inicial
localizacion(objetivos, real, ideal, ideal.pose(), 6, 1)
LIMIT = .85

for punto in objetivos:
  while distancia(tray_ideal[-1], punto) > EPSILON and len(tray_ideal) <= 1000:
    pose = ideal.pose()
    w = angulo_rel(pose, punto)
    if w > W:
        w = W
    if w < -W:
        w = -W
    v = distancia(pose, punto)
    if v > V:
        v = V
    if v < 0:
        v = 0
    if HOLONOMICO:
      if GIROPARADO and abs(w) > 0.01:
        v = 0
      ideal.move(w, v)
      real.move(w, v)
    else:
      ideal.move_triciclo(w, v, LONGITUD)
      real.move_triciclo(w, v, LONGITUD)
    tray_real.append(real.pose())
    tray_ideal.append(ideal.pose())

    # Relocalización en caso de superar el LIMIT
    err = ideal.measurement_prob(real.sense(objetivos), objetivos)
    if err > LIMIT:
      localizacion(objetivos, real, ideal, ideal.pose(), 1, 0)

    espacio += v
    tiempo += 1

if len(tray_ideal) > 1000:
  print("<!> Trayectoria muy larga - puede que no se haya alcanzado la posición final.")
print("Recorrido: " + str(round(espacio, 3)) + "m / " + str(tiempo / FPS) + "s")
print("Distancia real al objetivo: "+ str(round(distancia(tray_real[-1], objetivos[-1]), 3))+ "m")
mostrar(objetivos, tray_ideal, tray_real)  # Representación gráfica