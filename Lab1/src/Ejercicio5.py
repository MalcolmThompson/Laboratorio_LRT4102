from random import randint #Generamos números aleatorios con esto

# Generar un número aleatorio entre 1 y 10
numero_secreto = randint(1, 10)

# Contador de intentos
veces = 0

# Bucle infinito hasta que el usuario adivine el número
while True:
    try:
        # Incrementar el contador de intentos
        veces += 1

        # Pedir al usuario que ingrese un número
        suposicion = int(input("Introduce un número entre 1 y 10: "))

        # Comparar con el número secreto
        if suposicion > numero_secreto:
            print("El número secreto es menor.")
        elif suposicion < numero_secreto:
            print("El número secreto es mayor.")
        else:
            # Si el número es correcto, termina el juego
            print(f"¡Bien hecho! Adivinaste el número en {veces} intentos.")
            break  # Sale del bucle

    except ValueError:
        print("Entrada inválida, ingresa un número válido.")
