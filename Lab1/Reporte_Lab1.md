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

## **1. Clases y Objetos en Python**
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

La Programación Orientada a Objetos (POO) en Python, promueve la reutilización y reduce la complejidad de los programas. Aplicando encapsulamiento, herencia y polimorfismo, los desarrolladores pueden escribir código más limpio y estructurado.

# Solucion problemas

# Referencias
- Downey, A. (2015). Think Python: How to Think Like a Computer Scientist. O'Reilly Media.
- Shaw, Z. (2019). Learn Python 3 the Hard Way. Addison-Wesley.
- Zelle, J. (2016). Python Programming: An Introduction to Computer Science. Franklin, Beedle & Associates.
- Martelli, A., Ravenscroft, A., & Holden, S. (2020). Python in a Nutshell. O’Reilly Media.
- Sebesta, R. (2019). Concepts of Programming Languages. Pearson.
- Stroustrup, B. (2018). The C++ Programming Language. Addison-Wesley.


