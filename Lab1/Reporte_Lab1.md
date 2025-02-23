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
## **Funciones en Python**
Las funciones permiten reutilizar código agrupando una serie de instrucciones dentro de un bloque con nombre.

```python
def calcular_area_circulo(radio):
    return 3.1416 * radio ** 2

area = calcular_area_circulo(5)
print(f"El área del círculo es: {area}")
```
Este código define una función calcular_area_circulo() que recibe un valor radio y retorna el área de un círculo.

Python es un lenguaje poderoso y versátil que ofrece una sintaxis clara y flexible. Gracias a su tipado dinámico, estructuras de control y soporte para múltiples paradigmas, es utilizado en una amplia gama de aplicaciones, desde desarrollo web hasta inteligencia artificial. Su facilidad de uso y amplia comunidad lo convierten en una excelente opción tanto para principiantes como para programadores experimentados.

# Referencias
- Downey, A. (2015). Think Python: How to Think Like a Computer Scientist. O'Reilly Media.
- Shaw, Z. (2019). Learn Python 3 the Hard Way. Addison-Wesley.
- Zelle, J. (2016). Python Programming: An Introduction to Computer Science. Franklin, Beedle & Associates.


