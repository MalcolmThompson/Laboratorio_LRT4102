# Definir una lista de números
valores = [7, 1, 2, 6, 14, 35, 17, 28, 59, 4]

# Listas vacías para almacenar pares e impares
pares_lista = []
impares_lista = []

# Recorrer la lista y clasificar los números en pares e impares
for valor in valores:
    if valor % 2 == 0:  # Si el residuo es cero, es par
        pares_lista.append(valor)
    else:
        impares_lista.append(valor)  # Sino, es impar

# Calcular el promedio de los números pares
if len(pares_lista) > 0:
    suma_pares = sum(pares_lista)
    cantidad_pares = len(pares_lista)
    promedio_pares = suma_pares / cantidad_pares
else:
    promedio_pares = 0  # Evita la división por cero

# Calcular el producto de los números impares sin usar math.prod()
if len(impares_lista) > 0:
    producto_impares = 1  # Inicializamos en 1 porque multiplicar por 0 daría siempre 0
    for num in impares_lista:
        producto_impares *= num  # Multiplicamos cada número impar en la lista
else:
    producto_impares = 0  # Si no hay impares, el producto es 0

# Imprimir la información
print("Lista de números:", valores)
print("Números pares:", pares_lista)
print("Números impares:", impares_lista)
print("Promedio de pares:", promedio_pares)
print("Producto de impares:", producto_impares)
