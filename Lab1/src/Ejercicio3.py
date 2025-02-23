# Lista que almacenará la información de los operadores
operadores = [
    ["Juan", 15, 10],
    ["Enrique", 7, 55],
    ["Carlos", 19, 32],
    ["Maria", 11, 31],
    ["Henry", 10, 75],
    ["Malcolm", 21, 20]
]

# Imprimir encabezado
print("\nLista de sueldos de operadores:")

# Iterar sobre cada operador, calcular su sueldo y mostrar la información
for operador in operadores:
    nombre = operador[0]  # Nombre del operador
    sueldo_por_hora = operador[1]  # Sueldo por hora
    horas_trabajadas = operador[2]  # Horas trabajadas
    sueldo_total = sueldo_por_hora * horas_trabajadas  # Cálculo del sueldo total

    # Imprimir resultado formateado
    print(f"Operador: {nombre}, Sueldo total: ${sueldo_total:.2f}")
