import random

# Clase que representa un producto en el inventario
class Producto:
    def __init__(self, id_producto, nombre, precio, cantidad):
        """Inicializa un producto con su ID, nombre, precio y cantidad en stock."""
        self.id = id_producto
        self.nombre = nombre
        self.precio = precio
        self.stock = cantidad  

    def vender(self, cantidad):
        """Reduce la cantidad en stock cuando se realiza una venta."""
        if cantidad > self.stock:
            return False
        self.stock -= cantidad
        return True

    def info(self):
        """Devuelve una cadena con la información del producto."""
        estado = "Disponible" if self.stock > 0 else "No disponible"
        return f"ID: {self.id} | {self.nombre} | ${self.precio:.2f} | Stock: {self.stock} | Estado: {estado}"


# Clase que gestiona el inventario de productos
class Inventario:
    def __init__(self):
        """Inicializa un diccionario vacío para almacenar los productos."""
        self.productos = {}

    def agregar_producto(self, nombre, precio, cantidad):
        """Añade un producto con un ID único generado aleatoriamente."""
        id_producto = random.randint(1000, 9999)
        while id_producto in self.productos:
            id_producto = random.randint(1000, 9999)

        self.productos[id_producto] = Producto(id_producto, nombre, precio, cantidad)
        print(f"\nProducto añadido: {nombre} (ID: {id_producto})")

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

    def consultar_producto_agregado(self, id_producto):
        """Muestra toda la información de un producto si existe en el inventario."""
        producto = self.productos.get(id_producto)
        if producto:
            print("\nInformación del Producto:")
            print(f"ID: {producto.id}")
            print(f"Nombre: {producto.nombre}")
            print(f"Precio: ${producto.precio:.2f}")
            print(f"Stock: {producto.stock}")
        else:
            print("\nProducto no encontrado.")

    def calcular_valor_total(self):
        """Calcula el valor total del inventario sumando precio * cantidad de cada producto."""
        total = sum(p.precio * p.stock for p in self.productos.values())
        print(f"\nValor total del inventario: ${total:.2f}")

    def mostrar_productos(self):
        """Muestra todos los productos almacenados en el inventario."""
        if not self.productos:
            print("\nInventario vacío.")
        else:
            print("\nInventario:")
            for producto in self.productos.values():
                print(f"   {producto.info()}")


# Clase que maneja la interacción con el usuario
class Tienda:
    def __init__(self):
        """Inicializa la tienda creando un objeto de la clase Inventario y ejecutando el menú principal."""
        self.inventario = Inventario()
        self.ejecutar()

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

    def agregar(self):
        """Solicita los datos de un producto y lo agrega al inventario."""
        try:
            self.inventario.agregar_producto(
                input("\nNombre: "), float(input("Precio: ")), int(input("Cantidad: "))
            )
        except ValueError:
            print("\nEntrada inválida.")  

    def vender(self):
        """Solicita un ID de producto y una cantidad para realizar la venta."""
        try:
            self.inventario.vender_producto(int(input("\nID Producto: ")), int(input("Cantidad: ")))
        except ValueError:
            print("\nEntrada inválida.")

    def consultar_producto_agregado(self):
        """Solicita un ID de producto y muestra su ID, nombre, precio y stock."""
        try:
            id_producto = int(input("\nID Producto: "))
            self.inventario.consultar_producto_agregado(id_producto)
        except ValueError:
            print("\nEntrada inválida.")

    def mostrar(self):
        """Muestra todos los productos en el inventario."""
        self.inventario.mostrar_productos()

    def valor_total(self):
        """Muestra el valor total del inventario."""
        self.inventario.calcular_valor_total()

    def error(self):
        """Muestra un mensaje de error si el usuario ingresa una opción no válida."""
        print("\nOpción no válida.")

# Ejecutar el programa
Tienda()