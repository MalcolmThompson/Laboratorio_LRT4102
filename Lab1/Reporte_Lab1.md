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

