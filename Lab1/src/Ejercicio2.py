# Se pide al usuario que ingrese el número de horas trabajadas
horas = int(input("Ingresa el número de horas que has trabajado: "))

# Se pide al usuario que ingrese el costo por hora
costo = int(input("Ingrese el costo de cada hora trabajada: "))

# Calcular el salario total
salario = horas * costo

# Imprimir el resultado
print("El salario que te corresponde es de:", salario, "$")
