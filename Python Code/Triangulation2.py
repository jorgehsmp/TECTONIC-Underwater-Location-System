import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

# Definir las coordenadas de los tres puntos y las distancias al objeto
puntos = np.array([
    [45.779338836669918, 15.936838150024414],  # P1
    [45.779228210449218, 15.936708450317382],  # P2
    [45.779338836669918, 15.936790466308594]   # P3
])

distancias = np.array([4.32 + 0.81, 10.2 + 0.81, 1.98 + 0.81])  # Distancias ajustadas con el error

# Función para las ecuaciones a resolver
def equations(vars, *parameters):
    x, y = vars
    p, d = parameters
    eqs = [
        (x - p[0, 0])**2 + (y - p[0, 1])**2 - d[0]**2,
        (x - p[1, 0])**2 + (y - p[1, 1])**2 - d[1]**2,
    ]
    # Solo se usan dos ecuaciones, ya que hay dos incógnitas (x, y)
    return eqs

# Punto inicial para la búsqueda (promedio de las coordenadas de los puntos)
x0, y0 = puntos.mean(axis=0)

# Resolver el sistema de ecuaciones
solucion, infodict, ier, mesg = fsolve(equations, (x0, y0), args=(puntos, distancias), full_output=True)

# Verificar si la solución es exitosa
if ier == 1:
    print(f"Solución encontrada: {solucion}")
else:
    print("Solución no encontrada:", mesg)

# Visualización
fig, ax = plt.subplots()
for i, punto in enumerate(puntos):
    circulo = plt.Circle((punto[0], punto[1]), distancias[i], color='blue', fill=False)
    ax.add_artist(circulo)

# Dibujar los puntos y la solución
ax.plot(puntos[:, 0], puntos[:, 1], 'ro')  # Puntos
ax.plot(solucion[0], solucion[1], 'gx')  # Solución

plt.xlabel('X')
plt.ylabel('Y')
ax.set_aspect('equal', 'box')
plt.show()
