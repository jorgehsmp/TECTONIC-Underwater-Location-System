import folium

# Coordenadas de los puntos
puntos = [
    [45.779338836669918, 15.936838150024414],
    [45.779228210449218, 15.936708450317382],
    [45.779338836669918, 15.936790466308594]
]

# Crear un mapa centrado en la media de las coordenadas
mapa_centro = [sum(x)/len(puntos) for x in zip(*puntos)]
m = folium.Map(location=mapa_centro, zoom_start=18)  # Ajuste del zoom para mejor visualización

# Añadir puntos al mapa
for punto in puntos:
    folium.Marker(location=punto).add_to(m)

# Guardar el mapa en un archivo HTML
m.save("mi_mapa.html")

# Indicación para abrir el mapa
print("El mapa se ha guardado como 'mi_mapa.html'. Abre este archivo en un navegador web para ver el mapa.")
