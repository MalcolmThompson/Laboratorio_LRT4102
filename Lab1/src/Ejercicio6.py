import random

# Definir tamaño del mapa
tamaño = 5

# Crear la matriz del terreno con espacios libres representados por "o"
terreno = [['o' for _ in range(tamaño)] for _ in range(tamaño)]

# Definir posición inicial y destino
pos_actual = (0, 0)
pos_final = (tamaño - 1, tamaño - 1)

# Almacenar la ruta recorrida
trayectoria = []

# Direcciones de movimiento disponibles
movs = ['D', 'B', 'I', 'A']
direc = 'D'

# Generar obstáculos aleatorios en el mapa sin bloquear el inicio ni la meta
num_obstaculos = random.randint(tamaño, tamaño * 2)
for _ in range(num_obstaculos):
    while True:
        fila, col = random.randint(0, tamaño - 1), random.randint(0, tamaño - 1)
        if (fila, col) not in [(0, 0), pos_final]:  # Evitar bloquear la entrada y salida
            terreno[fila][col] = 'X'
            break

# Añadir posición inicial al camino recorrido
trayectoria.append(pos_actual)

# Diccionario de desplazamientos
desplazamiento = {
    'D': (0, 1),   # Derecha
    'I': (0, -1),  # Izquierda
    'A': (-1, 0),  # Arriba
    'B': (1, 0)    # Abajo
}

# Función anónima para verificar si un movimiento es válido
es_valido = lambda x, y: 0 <= x < tamaño and 0 <= y < tamaño and terreno[x][y] != 'X'

# Iniciar movimiento del robot
fila, col = pos_actual
while (fila, col) != pos_final:
    mov_exitoso = False
    dx, dy = desplazamiento[direc]
    nueva_fila, nueva_col = fila + dx, col + dy

    if es_valido(nueva_fila, nueva_col):
        fila, col = nueva_fila, nueva_col
        trayectoria.append((fila, col))
        mov_exitoso = True
    else:
        direc = movs[(movs.index(direc) + 1) % 4]  # Cambia de dirección en sentido horario

    if not mov_exitoso and not any(es_valido(fila + dx, col + dy) for dx, dy in desplazamiento.values()):
        print("\nNo hay ruta disponible.")
        break

# Mostrar resultados
if (fila, col) == pos_final:
    print("\nEl robot llegó al destino.")

# Imprimir el mapa del terreno con la trayectoria recorrida
print("\nMapa del Terreno:")
for i in range(tamaño):
    for j in range(tamaño):
        if (i, j) in trayectoria:
            print("*", end=" ")  # Marca el camino con "*"
        else:
            print(terreno[i][j], end=" ")
    print()

# Crear una nueva matriz para visualizar la trayectoria con flechas
mapa_flechas = [['o' for _ in range(tamaño)] for _ in range(tamaño)]
indicadores = {(0, 1): '→', (0, -1): '←', (-1, 0): '↑', (1, 0): '↓'}

for k in range(len(trayectoria) - 1):
    f1, c1 = trayectoria[k]
    f2, c2 = trayectoria[k + 1]
    dx, dy = f2 - f1, c2 - c1
    mapa_flechas[f1][c1] = indicadores[(dx, dy)]

mapa_flechas[pos_final[0]][pos_final[1]] = 'F'  # Marca la meta con 'F'

print("\nMapa con la Ruta Seguida:")
for fila in mapa_flechas:
    print(" ".join(fila))
