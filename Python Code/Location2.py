import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyproj import Proj, transform
from scipy.optimize import fsolve

# Define las coordenadas geográficas de los puntos (latitud, longitud)
puntos_lat_lon = [
    (45.779338836669918,15.936838150024414),
    (45.779228210449218,15.936708450317382),
    (45.779338836669918,15.936790466308594)
]

GPS_e = 0.81  # Error en las coordenadas GPS en metros
Acoustic_e = 0.1  # Error en la medición acústica en metros

r1 = 4.22 + GPS_e + Acoustic_e  # Radio de la esfera en metros, para un diámetro total de 3m
r2 = 10.1 + GPS_e + Acoustic_e
r3 = 1.88 + GPS_e + Acoustic_e

# Creando una malla de puntos para la esfera
phi, theta = np.mgrid[0.0:2 * np.pi:100j, 0.0:np.pi:50j]
x1 = r1 * np.sin(theta) * np.cos(phi)
y1 = r1 * np.sin(theta) * np.sin(phi)
z1 = r1 * np.cos(theta)

x2 = r2 * np.sin(theta) * np.cos(phi)
y2 = r2 * np.sin(theta) * np.sin(phi)
z2 = r2 * np.cos(theta)

x3 = r3 * np.sin(theta) * np.cos(phi)
y3 = r3 * np.sin(theta) * np.sin(phi)
z3 = r3 * np.cos(theta)

# Establecer la proyección UTM correspondiente a la zona de los puntos
# Nota: Deberías ajustar la zona UTM según tu ubicación específica
utm_zone = 33  # Zona UTM ejemplo para una parte de Europa
proj_utm = Proj(proj='utm', zone=utm_zone, ellps='WGS84')

# Convertir coordenadas geográficas (latitud, longitud) a UTM (X, Y)
puntos_utm = [transform(Proj(init='epsg:4326'), proj_utm, lon, lat) for lat, lon in puntos_lat_lon]

# Calcular el punto de origen como el primer punto para el desplazamiento
origen_x, origen_y = puntos_utm[0]

# Calcular coordenadas relativas en metros desde el origen
puntos_xy_rel = [(x - origen_x, y - origen_y) for x, y in puntos_utm]

# Crear figura para la visualización 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Añadir puntos a la gráfica
for i, (x_rel, y_rel) in enumerate(puntos_xy_rel):
    # Asumiendo una altura de 0 para la visualización
    ax.scatter(x_rel, y_rel, 0, label=f'Point {i+1}')

# Dibujando la esfera sobre el origen con color rojo y 50% de transparencia
for i, (x_rel, y_rel) in enumerate(puntos_xy_rel):
    if i == 0:
        ax.plot_surface(x1 + x_rel, y1 + y_rel, z1, color='blue', alpha=0.25)
    else:
        if i == 1:
            ax.plot_surface(x2 + x_rel, y2 + y_rel, z2, color='orange', alpha=0.25)
        else:
            if i == 2:
                ax.plot_surface(x3 + x_rel, y3 + y_rel, z3, color='green', alpha=0.25)



# Definir las coordenadas de los tres puntos y las distancias al objeto
puntos = np.array([
    [45.779338836669918, 15.936838150024414],  # P1
    [45.779228210449218, 15.936708450317382],  # P2
    [45.779338836669918, 15.936790466308594]   # P3
])

distancias = np.array([4.32 + 0.91, 10.2 + 0.91, 1.98 + 0.91])  # Distancias ajustadas con el error

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

# Dibujar los puntos y la solución
ax.plot(solucion[0], solucion[1], 'gx')  # Solución



# Configurar leyenda y etiquetas
ax.legend()
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Height (m)')

# Mostrar gráfica
plt.show()

# Función para calcular la distancia Euclidiana entre dos puntos en coordenadas UTM
def distancia_entre_puntos(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

# Calcular e imprimir las distancias entre los puntos
for i in range(len(puntos_utm) - 1):
    for j in range(i + 1, len(puntos_utm)):
        distancia = distancia_entre_puntos(puntos_utm[i], puntos_utm[j])
        print(f"Distancia entre Punto {i+1} y Punto {j+1}: {distancia:.2f} metros")