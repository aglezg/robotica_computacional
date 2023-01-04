# Cinemática Inversa
Esta práctica trata de resolver con un programa escrito en _python_ el problema de la cinemática inversa.

## Datos generales
- Universidad de La Laguna
- Curso nº 4
- Asignatura: Robótica computacional
- Autor: Adrián González Galván
- Fecha de entrega: 07/12/2022

## Ejecución del programa
### Parámetros de ejecución
El programa se ejecuta de la siguiente manera:
`$ python x y [input.txt]`, donde:
- `x` = Coordenada 'x' del punto de destino
- `y` = Coordenada 'y' del punto de destino
- `[input.txt]` = Fichero con opciones de configuración del brazo robótico
### Ejemplo de estructura del fichero de configuración
```
3           <-- Número de articulaciones
prismatic   <-- Tipo de articulación (rotation | prismatic)
0           <-- Grado inicial de la articulación
5           <-- Longitud inicial de la articulación
20          <-- Límite de la articulación
rotation
0
5
20
rotation
0
5
20
```
