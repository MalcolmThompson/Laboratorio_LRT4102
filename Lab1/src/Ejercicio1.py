import random

class Producto:
    def __init__(self, id_producto, nombre, precio, cantidad):
        self.id = id_producto
        self.nombre = nombre
        self.precio = precio
        self.stock = cantidad

    def vender(self, cantidad):
        if cantidad > self.stock:
            return False
        self.stock -= cantidad
        return True

    def info(self):
        return f"ID: {self.id} | {self.nombre} | ${self.precio:.2f} | Stock: {self.stock} | {'Disponible' if self.stock > 0 else 'Agotado'}"


class Inventario:
    def __init__(self):
        self.productos = {}

    def agregar_producto(self, nombre, precio, cantidad):
        id_producto = random.randint(1000, 9999)
        while id_producto in self.productos:
            id_producto = random.randint(1000, 9999)
        self.productos[id_producto] = Producto(id_producto, nombre, precio, cantidad)
        print(f"\n Producto añadido: {nombre} (ID: {id_producto})")

    def vender_producto(self, id_producto, cantidad):
        producto = self.productos.get(id_producto)
        if not producto:
            print("\n No existe un producto con ese ID.")
        elif producto.vender(cantidad):
            print(f"\n Venta realizada: {cantidad} unidades de '{producto.nombre}'.")
            if producto.stock == 0:
                print(f" '{producto.nombre}' se agotó y fue eliminado.")
                del self.productos[id_producto]
        else:
            print(f"\n Stock insuficiente. Quedan {producto.stock} unidades.")

    def consultar_producto(self, id_producto):
        print(f"\n {self.productos.get(id_producto, 'Producto no encontrado')}")

    def calcular_valor_total(self):
        total = sum(p.precio * p.stock for p in self.productos.values())
        print(f"\n Valor total del inventario: ${total:.2f}")

    def mostrar_productos(self):
        if not self.productos:
            print("\n Inventario vacío.")
        else:
            print("\n Inventario:")
            for producto in self.productos.values():
                print(f"   {producto.info()}")


class Tienda:
    def __init__(self):
        self.inventario = Inventario()
        self.ejecutar()

    def ejecutar(self):
        opciones = {
            "1": self.agregar,
            "2": self.vender,
            "3": self.consultar,
            "4": self.mostrar,
            "5": self.valor_total,
            "6": exit
        }
        while True:
            print("\n Menú:  1️ Agregar  2️ Vender  3️ Consultar  4️ Mostrar  5️ Valor  6️ Salir")
            opcion = input("Selecciona opción: ")
            opciones.get(opcion, self.error)()

    def agregar(self):
        try:
            self.inventario.agregar_producto(
                input("\n Nombre: "), float(input("$ Precio: ")), int(input(" Cantidad: "))
            )
        except ValueError:
            print("\n Entrada inválida.")

    def vender(self):
        try:
            self.inventario.vender_producto(int(input("\n ID Producto: ")), int(input(" Cantidad: ")))
        except ValueError:
            print("\n Entrada inválida.")

    def consultar(self):
        try:
            self.inventario.consultar_producto(int(input("\n ID Producto: ")))
        except ValueError:
            print("\n Entrada inválida.")

    def mostrar(self):
        self.inventario.mostrar_productos()

    def valor_total(self):
        self.inventario.calcular_valor_total()

    def error(self):
        print("\n Opción no válida.")

# Ejecutar el programa
Tienda()