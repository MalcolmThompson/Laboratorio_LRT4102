# Introducción
Python es un lenguaje de programación de alto nivel, dinámico e interpretado, ampliamente utilizado en diversos campos, desde desarrollo de software hasta inteligencia artificial y análisis de datos (Downey, 2015). Su diseño se centra en la legibilidad del código y la simplicidad de su sintaxis, lo que lo convierte en una herramienta accesible tanto para principiantes como para expertos.

Este lenguaje admite múltiples paradigmas de programación, incluyendo **programación estructurada, orientada a objetos y funcional**, lo que lo hace flexible para resolver distintos tipos de problemas (Zelle, 2016). Gracias a su extensa biblioteca estándar y su comunidad activa, Python se ha convertido en uno de los lenguajes más populares en el mundo de la tecnología.

---

## **1. Variables y Tipos de Datos en Python**

En Python, las variables se pueden crear sin necesidad de especificar un tipo de dato explícitamente, ya que el lenguaje cuenta con **tipado dinámico**, lo que significa que el tipo de dato se asigna en tiempo de ejecución según el valor proporcionado (Shaw, 2019).

A continuación, se presentan los tipos de datos fundamentales en Python:

- **Números enteros (`int`)**: Representan valores numéricos sin decimales, por ejemplo, `15`, `-8`, `1000`.
- **Números flotantes (`float`)**: Permiten representar valores con decimales, como `3.1416` o `-2.5`.
- **Cadenas de texto (`str`)**: Cualquier secuencia de caracteres encerrada entre comillas (`"Hola"`, `'Python'`).
- **Valores booleanos (`bool`)**: Solo pueden tener dos valores: `True` o `False`.
- **Listas (`list`)**: Estructuras ordenadas y modificables, por ejemplo, `[1, "rojo", 3.5]`.
- **Tuplas (`tuple`)**: Similares a las listas, pero inmutables: `(2, "azul", 9.1)`.
- **Diccionarios (`dict`)**: Almacenan pares clave-valor, como `{"nombre": "Luis", "edad": 25}`.

### **Ejemplo de Uso de Variables**
```python
numero = 42
pi = 3.14
mensaje = "Hola, Python!"
es_valido = True
colores = ["rojo", "verde", "azul"]
coordenadas = (10, 20)
usuario = {"nombre": "Ana", "edad": 30}

print(numero, pi, mensaje, es_valido, colores, coordenadas, usuario)
```
## **2. Control de Flujo en Python**
Python permite modificar el flujo de ejecución del código mediante estructuras condicionales y bucles, lo que facilita la toma de decisiones y la ejecución repetitiva de instrucciones.

### **Condicionales (if, elif, else)**
Los condicionales evalúan expresiones lógicas y ejecutan diferentes bloques de código según el resultado.

```python
temperatura = 25

if temperatura > 30:
    print("Hace mucho calor")
elif temperatura > 20:
    print("El clima es agradable")
else:
    print("Hace frío")
```
Este fragmento de código evalúa la temperatura y selecciona el mensaje correspondiente.

### **Bucles en python**
Los bucles permiten ejecutar instrucciones repetitivamente hasta que se cumpla una condición de finalización.

### **Bucle for**
El bucle for recorre elementos de una secuencia como listas o rangos numéricos.

```python
nombres = ["Carlos", "María", "Pedro"]
for nombre in nombres:
    print(f"Hola, {nombre}!")
```
### **Bucle while**
El bucle while ejecuta un bloque de código mientras una condición sea verdadera.

```python
contador = 0
while contador < 3:
    print(f"Intento número {contador + 1}")
    contador += 1
```
## **3. Funciones en Python**
Las funciones permiten reutilizar código agrupando una serie de instrucciones dentro de un bloque con nombre.

```python
def calcular_area_circulo(radio):
    return 3.1416 * radio ** 2

area = calcular_area_circulo(5)
print(f"El área del círculo es: {area}")
```
Este código define una función calcular_area_circulo() que recibe un valor radio y retorna el área de un círculo.

Python es un lenguaje poderoso y versátil que ofrece una sintaxis clara y flexible. Gracias a su tipado dinámico, estructuras de control y soporte para múltiples paradigmas, es utilizado en una amplia gama de aplicaciones, desde desarrollo web hasta inteligencia artificial. Su facilidad de uso y amplia comunidad lo convierten en una excelente opción tanto para principiantes como para programadores experimentados.

# Programación Orientada a Objetos

La **Programación Orientada a Objetos (POO)** es un paradigma de programación basado en la creación de **objetos** que encapsulan datos y comportamientos en un solo componente. Esta metodología facilita la organización del código, mejora la reutilización y permite modelar problemas de manera más intuitiva (Sebesta, 2019). 

Python es un lenguaje que **implementa POO de forma nativa**, permitiendo a los desarrolladores estructurar aplicaciones de manera eficiente mediante la creación de **clases y objetos**. Además, incorpora mecanismos como **encapsulamiento, herencia y polimorfismo**, los cuales optimizan el mantenimiento del software y reducen la redundancia en el código (Martelli et al., 2020).

---

## **1. Clases y Objetos**
En POO, una **clase** define una estructura base a partir de la cual se pueden crear **objetos** con características y comportamientos específicos (Stroustrup, 2018). 

Un **objeto** es una instancia de una clase, lo que significa que comparte sus atributos y métodos definidos. Python permite definir clases de manera sencilla utilizando la palabra clave `class`.

```python
class Vehiculo:
    def __init__(self, marca, modelo):
        self.marca = marca
        self.modelo = modelo

# Creación de un objeto a partir de la clase
auto = Vehiculo("Toyota", "Corolla")
```
Aquí, Vehiculo define una estructura básica con atributos marca y modelo, y auto es una instancia con valores específicos.

## **2. Principios Fundamentales de POO en Python**
Python implementa varios principios de POO que permiten mejorar la estructura y eficiencia del código.

### **2.1. Encapsulamiento**
El encapsulamiento consiste en proteger los datos de un objeto para evitar accesos o modificaciones no controladas. Python permite definir atributos públicos, protegidos y privados mediante convenciones de nombres.

```python
class Banco:
    def __init__(self, saldo):
        self.__saldo = saldo  # Atributo privado

    def ver_saldo(self):
        return self.__saldo  # Método de acceso controlado
```
El atributo __saldo es privado y solo puede ser accedido mediante métodos específicos dentro de la misma clase.

### **2.2. Herencia**
La herencia permite que una clase reutilice atributos y métodos de otra clase existente, evitando la duplicación de código y promoviendo la modularidad (Downey, 2021).

```python
class Persona:
    def __init__(self, nombre):
        self.nombre = nombre

class Estudiante(Persona):
    pass  # Hereda todo de la clase Persona

alumno = Estudiante("Laura")
```
La clase Estudiante hereda los atributos de Persona sin necesidad de reescribirlos.

### **2.3. Polimorfismo**
El polimorfismo permite que múltiples clases utilicen un mismo método con distintas implementaciones, lo que mejora la flexibilidad del código.

```python
class Guitarra:
    def sonido(self):
        return "Suena una melodía."

class Bateria:
    def sonido(self):
        return "Produce un ritmo."

instrumentos = [Guitarra(), Bateria()]

for i in instrumentos:
    print(i.sonido())
```
Ambas clases tienen el método sonido(), pero cada una lo ejecuta de manera diferente.

# Solucion problemas

## **Problema 1**
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestre 
en pantalla la suma de todos los enteros desde 1 hasta n . La suma de los primeros enteros 
positivos puede ser calculada usando: suma = [n(n+1)]/2

### **Solución**
```python
# Pedir al usuario que ingrese un número entero
numero = int(input("¿Cuál es el número?: "))

# Calcular la suma de los primeros n números naturales usando la fórmula: n(n + 1)/2
suma = (numero * (numero + 1)) / 2

# Imprimir el resultado
print("La suma de todos los números es:", suma)
```
### **Breve descripción**
Este código solicita al usuario que ingrese un número entero y, utilizando la fórmula matemática [n(n+1)]/2, calcula la suma de los primeros n números naturales. Posteriormente, imprime el resultado en pantalla, mostrando la suma total de los valores desde 1 hasta el número ingresado. Su propósito es ilustrar un cálculo matemático eficiente sin necesidad de bucles.

## **Problema 2**
Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora. Después debe mostrar por pantalla la paga que le corresponde.

### **Solución**
```python
# Se pide al usuario que ingrese el número de horas trabajadas
horas = int(input("Ingresa el número de horas que has trabajado: "))

# Se pide al usuario que ingrese el costo por hora
costo = int(input("Ingrese el costo de cada hora trabajada: "))

# Calcular el salario total
salario = horas * costo

# Imprimir el resultado
print("El salario que te corresponde es de:", salario, "$")
```
### **Breve descripción**
Este código solicita al usuario que ingrese la cantidad de horas trabajadas y el costo por hora, luego multiplica ambos valores para calcular el salario total. Finalmente, muestra el resultado en pantalla, indicando el monto que corresponde al trabajador en función del tiempo laborado y la tarifa establecida por hora.

## **Problema 3**
Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores. Imprime el nombre y el sueldo a pagar de cada operador.

### **Solución**
```python
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
```
### **Breve descripción**
Este código almacena en una lista la información de varios operadores, incluyendo su nombre, sueldo por hora y cantidad de horas trabajadas. Luego, recorre la lista y para cada operador, calcula su sueldo total multiplicando el sueldo por hora por las horas trabajadas. Finalmente, imprime en pantalla el nombre del operador junto con el monto correspondiente a su salario, formateado con dos decimales para mayor claridad.

## **Problema 4**
Crea una lista llamada numeros que contenga al menos 10 números. Calcula el promedio de los números pares y el producto de los números impares. Imprime los resultados.

### **Solución**
```python
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
```
### **Breve descripción**
Este código define una lista de números y los clasifica en dos listas separadas: una para los números pares y otra para los impares. Luego, calcula el promedio de los números pares sumando sus valores y dividiéndolos por la cantidad de elementos, asegurándose de evitar una división por cero. Posteriormente, obtiene el producto de los números impares multiplicando todos sus valores sin utilizar librerias externas. Finalmente, imprime en pantalla la lista original junto con los números pares, impares, su promedio y el producto de los impares.

## **Problema 5**
Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe proporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos intentos el usuario logró adivinar el número.

### **Solución**
```python
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
```
### **Breve descripción**
Este código implementa un juego de adivinanza de números en el que el programa genera un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. Se inicializa un contador de intentos, y en un bucle infinito, el usuario introduce un número. Si el número ingresado es mayor o menor que el número secreto, el programa da una pista. Si el usuario acierta, se muestra un mensaje indicando el número de intentos realizados y el juego finaliza. Además, el código maneja errores en caso de que el usuario introduzca un valor no numérico, asegurando una ejecución robusta.

## **Problema 6**
El programa debe generar una matriz de al menos 5x5. El robot inicia su camino en la posición (0,0) de la matriz y debe salir en la posición (4,4) o la máxima posición si se cambia el tamaño de matriz. El numero y la posición de los obstáculos es aleatoria. El robot solo puede avanzar, girar a la izquierda o a la derecha para buscar un camino libre, en el eventual caso que el robot no pueda salir debe imprimir en pantalla “Imposible llegar al destino”. En caso de que el robot llegue a su destino final deberá imprimir el mapa, con los espacios libres y obstáculos de la siguiente forma X obstáculo o libre. Deberá imprimir también la ruta que siguió. Mostrar un segundo mapa con el “camino” seguido por el robot mediante flechas 

### **Solución**
```python
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
```
### **Breve descripción**
Este código simula un robot explorador en un terreno representado por una matriz de tamaño 5x5. El robot comienza en la posición (0,0) y debe encontrar un camino hasta la meta en la esquina inferior derecha. A lo largo del mapa, se generan obstáculos aleatorios (X) que el robot debe evitar.

El robot sigue una estrategia de exploración moviéndose en cuatro direcciones posibles (D: derecha, B: abajo, I: izquierda, A: arriba). Si encuentra un obstáculo o un límite del mapa, cambia su dirección en sentido horario hasta hallar un camino libre.

La trayectoria recorrida se almacena en una lista y se representa visualmente en un mapa con *, mientras que en otro mapa se usan flechas (→, ↓, ↑, ←) para indicar la dirección del movimiento. Si el robot no encuentra salida, el programa imprime "No hay ruta disponible.". En caso de éxito, muestra "El robot llegó al destino.", junto con el mapa de su recorrido.

## **Problema 7**
Una tienda quiere gestionar su inventario de productos. Para ello, debes implementar un sistema 
en Python que permita: 
- Crear productos, cada uno con un nombre, precio y cantidad en stock. 
- Actualizar la cantidad en stock cuando se venden productos. 
- Mostrar la información de un producto con su disponibilidad. 
- Calcular el valor total del inventario (precio × cantidad de cada producto).

### **Solución explicada**
Este código implementa un sistema de inventario de productos usando Programación Orientada a Objetos (POO). El programa permite agregar, vender, consultar y visualizar productos, asegurando que el inventario se gestione correctamente.

#### **Importación de librerias**
```python
import random
```
import random: Se importa la librería random, que será utilizada para generar IDs aleatorios para los productos dentro del inventario.

#### **Clase Producto: Representación de un Producto**
Esta clase define la estructura básica de un producto, incluyendo ID, nombre, precio y cantidad en stock.

```python
class Producto:
    def __init__(self, id_producto, nombre, precio, cantidad):
        """Inicializa un producto con su ID, nombre, precio y cantidad en stock."""
        self.id = id_producto
        self.nombre = nombre
        self.precio = precio
        self.stock = cantidad  
```
- self.id: Almacena el identificador único del producto.
- self.nombre: Guarda el nombre del producto.
- self.precio: Representa el precio unitario del producto.
- self.stock: Indica la cantidad disponible del producto en el inventario.

#### **Método vender(): Reduce el Stock al Vender un Producto**
```python
def vender(self, cantidad):
    """Reduce la cantidad en stock cuando se realiza una venta."""
    if cantidad > self.stock:
        return False  # No hay suficiente stock para la venta
    self.stock -= cantidad  # Se descuenta la cantidad vendida del stock
    return True  # Indica que la venta fue exitosa
```
- Verifica si hay suficiente stock disponible antes de proceder con la venta.
- Si hay stock suficiente, lo reduce y devuelve True.
- Si no hay stock suficiente, devuelve False.

#### **Método info(): Muestra Información del Producto**
```python
def info(self):
    """Devuelve una cadena con la información del producto."""
    estado = "Disponible" if self.stock > 0 else "No disponible"
    return f"ID: {self.id} | {self.nombre} | ${self.precio:.2f} | Stock: {self.stock} | Estado: {estado}"
```
- Retorna una cadena con los detalles del producto.
- Si el stock es mayor a 0, el estado será "Disponible", de lo contrario "No disponible".

#### **Clase Inventario: Gestión de Productos**
Esta clase maneja la agregación, venta, consulta y visualización de productos en el inventario.
```python
class Inventario:
    def __init__(self):
        """Inicializa un diccionario vacío para almacenar los productos."""
        self.productos = {}
```
- self.productos: Un diccionario que almacena los productos en formato {ID: Producto}.

#### **Método agregar_producto(): Añade Productos al Inventario**
```python
def agregar_producto(self, nombre, precio, cantidad):
    """Añade un producto con un ID único generado aleatoriamente."""
    id_producto = random.randint(1000, 9999)  # Genera un ID aleatorio único
    while id_producto in self.productos:
        id_producto = random.randint(1000, 9999)

    self.productos[id_producto] = Producto(id_producto, nombre, precio, cantidad)
    print(f"\nProducto añadido: {nombre} (ID: {id_producto})")
```
- Genera un ID aleatorio entre 1000 y 9999.
- Verifica que el ID sea único antes de agregarlo al diccionario.
- Crea una instancia de Producto y la almacena en self.productos.

#### **Método vender_producto(): Reduce Stock o Elimina Productos**
```python
def vender_producto(self, id_producto, cantidad):
    """Vende un producto si existe en el inventario y hay suficiente stock disponible."""
    producto = self.productos.get(id_producto)
    if not producto:
        print("\nNo existe un producto con ese ID.")
    elif producto.vender(cantidad):
        print(f"\nVenta realizada: {cantidad} unidades de '{producto.nombre}'.")
        if producto.stock == 0:
            print(f"'{producto.nombre}' se agotó y fue eliminado.")
            del self.productos[id_producto]
    else:
        print(f"\nStock insuficiente. Quedan {producto.stock} unidades.")
```
- Busca el producto en el inventario.
- Si existe, intenta vender la cantidad indicada.
- Si el stock llega a 0, elimina el producto del inventario.

#### **Clase Tienda: Manejo del Menú e Interacción con el Usuario**
Esta clase se encarga de gestionar la interacción con el usuario a través de un menú.
```python
class Tienda:
    def __init__(self):
        """Inicializa la tienda creando un objeto de la clase Inventario y ejecutando el menú principal."""
        self.inventario = Inventario()
        self.ejecutar()
```
- Crea una instancia de Inventario.
- Ejecuta el menú al inicializarse.

#### **Método ejecutar(): Menú Principal**
```python
def ejecutar(self):
    """Ejecuta el menú de opciones y permite la interacción con el usuario."""
    opciones = {
        "1": self.agregar,
        "2": self.vender,
        "3": self.consultar_producto_agregado,
        "4": self.mostrar,
        "5": self.valor_total,
        "6": exit
    }
    while True:
        print("\nMenú:")
        print(" 1. Agregar producto")
        print(" 2. Vender producto")
        print(" 3. Consultar producto agregado (ID, nombre, precio y stock)")
        print(" 4. Mostrar todos los productos")
        print(" 5. Ver valor total del inventario")
        print(" 6. Salir")

        opcion = input("Selecciona una opción: ")
        opciones.get(opcion, self.error)()
```
- Muestra las opciones disponibles en el sistema.
- Asocia cada opción a un método mediante un diccionario.
- Ejecuta la función correspondiente según la opción seleccionada.

#### **Ejecución del Programa**
```python
# Ejecutar el programa
Tienda()
```
- Inicia el programa creando una instancia de Tienda.
- El menú se activa automáticamente, permitiendo la interacción con el usuario.

# Librerias usadas

En este laboratorio se utilizó la libreria de random la cual nos permite generar números aleatorios.

# Referencias
- Downey, A. (2015). Think Python: How to Think Like a Computer Scientist. O'Reilly Media.
- Shaw, Z. (2019). Learn Python 3 the Hard Way. Addison-Wesley.
- Zelle, J. (2016). Python Programming: An Introduction to Computer Science. Franklin, Beedle & Associates.
- Martelli, A., Ravenscroft, A., & Holden, S. (2020). Python in a Nutshell. O’Reilly Media.
- Sebesta, R. (2019). Concepts of Programming Languages. Pearson.
- Stroustrup, B. (2018). The C++ Programming Language. Addison-Wesley.


