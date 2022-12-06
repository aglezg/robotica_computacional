#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure(1)
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
  raw_input()
  plt.clf()

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)
  
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# Valores articulares
articulations = 0                            # Número de articulaciones
th=[]                                # Ángulos
a =[]                                # Logitud de articulaciones
type = [] # Tipo de articulación
limits = []                        # Límites de las articulaciones


EPSILON = .01

plt.ion() # modo interactivo

# Introducción del punto para la cinemática inversa
if len(sys.argv) != 4:
  sys.exit("python " + sys.argv[0] + "[input.txt]")
objetivo=[float(sys.argv[1]), float(sys.argv[2])]

# Lectura de  los datos ********************************************************
input = open(sys.argv[3])
reader = input.readline()
reader = reader[:-1]
if (reader.isdigit() == False):
   print ("READER == ", reader)
   print("El número de articulaciones no es un dígito numérico, revise el fichero de entrada")
   exit(1)
articulations = int(reader)

for i in range(articulations):
  reader = input.readline() # Tipo de articulación
  reader = reader[:-1]
  print ("reader == ", reader)
  if (reader != "prismatic" and reader != "rotation"):
    print("El tipo de articulación debe ser 'prismatic' o 'rotation'")
    exit(1)
  type.append(reader)
  
  reader = input.readline() # Angulo inicial de la articulacion
  reader = reader[:-1]
  if (reader.isdigit() == False or int(reader) < 0):
    print("Los ángulos de las articulaciones deben ser numéricos y mayores que cero")
  th.append(int(reader))

  reader = input.readline() # Longitud inicial de la articulacion
  reader = reader[:-1]
  if (reader.isdigit() == False or int(reader) < 0):
    print("Las longitudes de las articulaciones deben ser numéricas y mayores que cero")
  a.append(int(reader))

  reader = input.readline() # Limite de la articulacion
  reader = reader[:-1]
  if (reader.isdigit() == False or int(reader) < 0):
    print("Los límites de las articulaciones deben ser numéricos y mayores que cero")
  limits.append(int(reader))

L = sum(a) # variable para representación gráfica

# Cálculos ********************************************************

O=range(len(th)+1) # Reservamos estructura en memoria
O[0]=cin_dir(th,a) # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O[0])

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  # Para cada combinación de articulaciones:
  for i in range(len(th)):
    # cálculo de la cinemática inversa:
    if (type[-1 -i] == "rotation"):
      # Cálculo del primer ángulo
      oppositeAlpha = objetivo[1] - O[i][-2 -i][1]
      contiguousAlpha = objetivo[0] - O[i][-2 -i][0]

      alpha = np.arctan2(oppositeAlpha, contiguousAlpha)

      # Cálculo del segundo ángulo
      oppositeAlpha2 = O[i][-1][1] - O[i][-2 -i][1]
      contiguousAlpha2 = O[i][-1][0] - O[i][-2 -i][0]

      alpha2 = np.arctan2(oppositeAlpha2, contiguousAlpha2)

      # Actualización de theta
      th[-1 - i] += (alpha - alpha2)
  
    elif (type[-1 -i] == "prismatic"):
      # Cálculo de ángulo 'w'
      w = 0
      for j in range(-1 -i):
        w += th[j]
      
      # Cálculo de 'd'
      d = np.dot([cos(w), sin(w)], [objetivo[0] - O[i][-1][0], objetivo[1] - O[i][-1][1]])

      # Actualizamos 'a'
      a[-1 -i] += d

      # Actualizamos L
      L = sum(a)
    else:
      print ("El tipo no es 'rotation' o 'prismatic'")
      exit(1)
    
    # Actualización de O
    O[i+1] = cin_dir(th,a)
    
  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))

# Cierre del fichero de lectura
input.close()