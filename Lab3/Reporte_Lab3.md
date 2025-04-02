# Introduction
Python is a high-level, dynamic, interpreted programming language widely used in various fields, from software development to artificial intelligence and data analysis (Downey, 2015). Its design focuses on the readability of the code and the simplicity of its syntax, making it an accessible tool for both beginners and experts.

This language supports multiple programming paradigms, including structured, object-oriented, and functional programming, making it flexible to solve different types of problems (Zelle, 2016). Thanks to its extensive standard library and active community, Python has become one of the most popular languages ​​in the world of technology.

---

## **1. Variables and Data Types in Python**

In Python, variables can be created without needing to specify a data type explicitly, since the language has **dynamic typing**, which means that the data type is assigned at run time based on the provided value (Shaw, 2019).

Below are the fundamental data types in Python:

- **Integer numbers (`int`)**: Represent numerical values ​​without decimals, for example, `15`, `-8`, `1000`.
- **Float numbers (`float`)**: They allow values ​​to be represented with decimals, such as `3.1416` or `-2.5`.
- **Text strings (`str`)**: Any sequence of characters enclosed in quotes (`"Hello"`, `'Python'`).
- **Boolean values ​​(`bool`)**: They can only have two values: `True` or `False`.
- **Lists (`list`)**: Ordered and modifiable structures, for example, `[1, "red", 3.5]`.
- **Tuples (`tuple`)**: Similar to lists, but immutable: `(2, "blue", 9.1)`.
- **Dictionaries (`dict`)**: They store key-value pairs, such as `{"name": "Luis", "age": 25}`.

### **Example of Variable Usage**
```python
number = 42
pi = 3.14
message = "Hello, Python!"
is_valid = True
colors = ["red", "green", "blue"]
coordinates = (10, 20)
user = {"name": "Ana", "age": 30}

print(number, pi, message, is_valid, colors, coordinates, user)
```
## **2. Flow Control in Python**
Python allows modifying the execution flow of the code through conditional structures and loops, which facilitate decision-making and repetitive execution of instructions.

### **Conditionals (if, elif, else)**
Conditionals evaluate logical expressions and execute different code blocks depending on the result.
```python
temperature = 25

if temperature > 30:
    print("It's very hot")
elif temperature > 20:
    print("The weather is pleasant")
else:
    print("It's cold")
```
This code snippet evaluates the temperature and selects the corresponding message.

### **Loops in Python**
Loops allow executing instructions repeatedly until a termination condition is met.

### **For Loop**
The for loop iterates over elements in a sequence, such as lists or numerical ranges.

```python
names = ["Carlos", "María", "Pedro"]
for name in names:
    print(f"Hello, {name}!")
```
### **While Loop**
The while loop executes a block of code while a condition remains true.

```python
counter = 0
while counter < 3:
    print(f"Attempt number {counter + 1}")
    counter += 1
```
## **3. Functions in Python**
Functions allow reusing code by grouping a series of instructions inside a named block.

```python
def calculate_circle_area(radius):
    return 3.1416 * radius ** 2

area = calculate_circle_area(5)
print(f"The area of the circle is: {area}")
```
This code defines a function calculate_circle_area() that takes a radius value and returns the area of a circle.

Python is a powerful and versatile language that offers clear and flexible syntax. Thanks to its dynamic typing, control structures, and support for multiple paradigms, it is used in a wide range of applications, from web development to artificial intelligence. Its ease of use and large community make it an excellent choice for both beginners and experienced programmers.

# Object-Oriented Programming (OOP)

**Object-Oriented Programming (OOP)** is a programming paradigm based on creating **objects** that encapsulate data and behaviors into a single component. This methodology facilitates **code organization, enhances reusability, and allows for a more intuitive modeling of problems** (Sebesta, 2019). 

Python is a language that **natively supports OOP**, enabling developers to efficiently structure applications through **classes and objects**. Additionally, it incorporates mechanisms such as **encapsulation, inheritance, and polymorphism**, which optimize software maintenance and reduce code redundancy (Martelli et al., 2020).

---

## **1. Classes and Objects**
In OOP, a **class** defines a base structure from which **objects** can be created, each with specific attributes and behaviors (Stroustrup, 2018). 

An **object** is an instance of a class, meaning it shares the attributes and methods defined within it. Python allows defining classes easily using the `class` keyword.

```python
class Vehicle:
    def __init__(self, brand, model):
        self.brand = brand
        self.model = model

# Creating an object from the class
car = Vehicle("Toyota", "Corolla")
```
Here, Vehicle defines a basic structure with brand and model attributes, and car is an instance with specific values.

## **2. Fundamental OOP Principles in Python**
Python implements several OOP principles that improve code structure and efficiency.

### **2.1. Encapsulation**
Encapsulation consists of protecting an object's data to prevent uncontrolled access or modifications. Python allows defining public, protected, and private attributes through naming conventions.

```python
class Bank:
    def __init__(self, balance):
        self.__balance = balance  # Private attribute

    def get_balance(self):
        return self.__balance  # Controlled access method
```
The attribute __balance is private and can only be accessed through specific methods within the class.

### **2.2. Inheritance**
Inheritance allows one class to reuse attributes and methods from another existing class, preventing code duplication and promoting modularity (Downey, 2021).

```python
class Person:
    def __init__(self, name):
        self.name = name

class Student(Person):
    pass  # Inherits everything from the Person class

student = Student("Laura")
```
The Student class inherits the attributes from Person without needing to rewrite them.

### **2.3. Polymorphism**
Polymorphism allows multiple classes to use the same method with different implementations, improving code flexibility.

```python
class Guitar:
    def sound(self):
        return "Plays a melody."

class Drums:
    def sound(self):
        return "Produces a rhythm."

instruments = [Guitar(), Drums()]

for i in instruments:
    print(i.sound())
```
Both classes have the sound() method, but each executes it differently.

# Problem Solutions

## **Problem 1**
Write a program that reads a positive integer "n" entered by the user and then displays on the screen the sum of all integers from 1 to n. The sum of the first positive integers can be calculated using: sum = [n(n+1)]/2

### **Solution**
```python
# Ask the user to enter an integer
number = int(input("What is the number?: "))

# Calculate the sum of the first n natural numbers using the formula: n(n + 1)/2
sum_result = (number * (number + 1)) / 2

# Print the result
print("The sum of all numbers is:", sum_result)
```
### **Brief Description**
This code prompts the user to enter an integer and, using the mathematical formula [n(n+1)]/2, calculates the sum of the first n natural numbers. It then prints the result on the screen, displaying the total sum of values from 1 to the entered number. Its purpose is to illustrate an efficient mathematical calculation without the need for loops.

## **Problem 2**
Write a program that asks the user for the number of hours worked and the hourly rate. Then, it should display the corresponding payment on the screen.

### **Solution**
```python
# Ask the user to enter the number of hours worked
hours = int(input("Enter the number of hours you have worked: "))

# Ask the user to enter the hourly rate
rate = int(input("Enter the hourly rate: "))

# Calculate the total salary
salary = hours * rate

# Print the result
print("Your total salary is:", salary, "$")
```
### **Brief Description**
This code prompts the user to enter the number of hours worked and the hourly rate, then multiplies both values to calculate the total salary. Finally, it displays the result on the screen, indicating the amount the worker is entitled to based on the time worked and the established hourly rate.

## **Problem 3**
Create a list containing the name, hourly wage, and hours worked for at least six operators. Print the name and total salary for each operator.

### **Solution**
```python
# List to store operator information
operators = [
    ["Juan", 15, 10],
    ["Enrique", 7, 55],
    ["Carlos", 19, 32],
    ["Maria", 11, 31],
    ["Henry", 10, 75],
    ["Malcolm", 21, 20]
]

# Print header
print("\nList of operator salaries:")

# Iterate over each operator, calculate their salary, and display the information
for operator in operators:
    name = operator[0]  # Operator's name
    hourly_wage = operator[1]  # Hourly wage
    hours_worked = operator[2]  # Hours worked
    total_salary = hourly_wage * hours_worked  # Total salary calculation

    # Print formatted result
    print(f"Operator: {name}, Total salary: ${total_salary:.2f}")
```
### **Brief Description**
This code stores information about several operators in a list, including their name, hourly wage, and hours worked. Then, it iterates through the list and, for each operator, calculates their total salary by multiplying the hourly wage by the hours worked. Finally, it prints the operator’s name along with the corresponding salary amount, formatted with two decimal places for clarity.

## **Problem 4**
Create a list named `numbers` containing at least 10 numbers. Calculate the average of the even numbers and the product of the odd numbers. Print the results.

### **Solution**
```python
# Define a list of numbers
values = [7, 1, 2, 6, 14, 35, 17, 28, 59, 4]

# Empty lists to store even and odd numbers
even_numbers = []
odd_numbers = []

# Iterate through the list and classify numbers as even or odd
for value in values:
    if value % 2 == 0:  # If the remainder is zero, it is even
        even_numbers.append(value)
    else:
        odd_numbers.append(value)  # Otherwise, it is odd

# Calculate the average of the even numbers
if len(even_numbers) > 0:
    sum_evens = sum(even_numbers)
    count_evens = len(even_numbers)
    average_evens = sum_evens / count_evens
else:
    average_evens = 0  # Prevents division by zero

# Calculate the product of the odd numbers without using math.prod()
if len(odd_numbers) > 0:
    product_odds = 1  # Initialized to 1 because multiplying by 0 would always give 0
    for num in odd_numbers:
        product_odds *= num  # Multiply each odd number in the list
else:
    product_odds = 0  # If there are no odd numbers, the product is 0

# Print the information
print("List of numbers:", values)
print("Even numbers:", even_numbers)
print("Odd numbers:", odd_numbers)
print("Average of evens:", average_evens)
print("Product of odds:", product_odds)
```
### **Brief Description**
This code defines a list of numbers and classifies them into two separate lists: one for even numbers and another for odd numbers. It then calculates the average of the even numbers by summing their values and dividing them by the number of elements, ensuring that division by zero is avoided. Next, it calculates the product of the odd numbers by multiplying all their values without using external libraries. Finally, it prints the original list along with the even numbers, odd numbers, their average, and the product of the odd numbers.

## **Problem 5**
Create a program that asks the user to guess a secret number. The program must generate a random number between 1 and 10, and the user must try to guess it. The program should provide hints if the number entered by the user is too high or too low. The while loop must continue until the user guesses correctly. At the end, it should print how many attempts it took for the user to guess the number.

### **Solution**
```python
from random import randint  # Generates random numbers

# Generate a random number between 1 and 10
secret_number = randint(1, 10)

# Attempt counter
attempts = 0

# Infinite loop until the user guesses the number
while True:
    try:
        # Increment the attempt counter
        attempts += 1

        # Ask the user to enter a number
        guess = int(input("Enter a number between 1 and 10: "))

        # Compare with the secret number
        if guess > secret_number:
            print("The secret number is lower.")
        elif guess < secret_number:
            print("The secret number is higher.")
        else:
            # If the number is correct, end the game
            print(f"Well done! You guessed the number in {attempts} attempts.")
            break  # Exit the loop

    except ValueError:
        print("Invalid input, please enter a valid number.")
```
### **Brief Description**
This code implements a number guessing game, where the program generates a random number between 1 and 10, and the user must try to guess it. An attempt counter is initialized, and within an infinite loop, the user inputs a number. If the number entered is higher or lower than the secret number, the program provides a hint. If the user guesses correctly, a message is displayed indicating the number of attempts taken, and the game ends. Additionally, the code includes error handling to prevent crashes if the user enters a non-numeric value.

## **Problem 6**
The program must generate a matrix of at least 5x5. The robot starts at position (0,0) and must exit at position (4,4) or the maximum position if the matrix size is changed. The number and position of obstacles are random. The robot can only move forward, turn left, or turn right to find an open path. If the robot cannot exit, it must print "No route available." If the robot reaches its final destination, it should print the map with free spaces and obstacles in the following format, where X represents an obstacle and o represents a free space. Additionally, the program must display the route followed by the robot with arrows.

### **Solution**
```python
import random

# Define the map size
size = 5

# Create a terrain matrix with open spaces represented by "o"
terrain = [['o' for _ in range(size)] for _ in range(size)]

# Define the initial position and destination
current_pos = (0, 0)
goal_pos = (size - 1, size - 1)

# Store the traveled path
path = []

# Available movement directions
directions = ['R', 'D', 'L', 'U']
current_dir = 'R'

# Generate random obstacles on the map without blocking the start or end positions
num_obstacles = random.randint(size, size * 2)
for _ in range(num_obstacles):
    while True:
        row, col = random.randint(0, size - 1), random.randint(0, size - 1)
        if (row, col) not in [(0, 0), goal_pos]:  # Avoid blocking entry and exit
            terrain[row][col] = 'X'
            break

# Add the initial position to the path
path.append(current_pos)

# Dictionary of movements
movement = {
    'R': (0, 1),   # Right
    'L': (0, -1),  # Left
    'U': (-1, 0),  # Up
    'D': (1, 0)    # Down
}

# Lambda function to verify if a movement is valid
is_valid = lambda x, y: 0 <= x < size and 0 <= y < size and terrain[x][y] != 'X'

# Start robot movement
row, col = current_pos
while (row, col) != goal_pos:
    successful_move = False
    dx, dy = movement[current_dir]
    new_row, new_col = row + dx, col + dy

    if is_valid(new_row, new_col):
        row, col = new_row, new_col
        path.append((row, col))
        successful_move = True
    else:
        current_dir = directions[(directions.index(current_dir) + 1) % 4]  # Rotate direction clockwise

    if not successful_move and not any(is_valid(row + dx, col + dy) for dx, dy in movement.values()):
        print("\nNo route available.")
        break

# Display results
if (row, col) == goal_pos:
    print("\nThe robot reached the destination.")

# Print the terrain map with the traveled path
print("\nTerrain Map:")
for i in range(size):
    for j in range(size):
        if (i, j) in path:
            print("*", end=" ")  # Mark the path with "*"
        else:
            print(terrain[i][j], end=" ")
    print()

# Create a new matrix to visualize the path with arrows
arrow_map = [['o' for _ in range(size)] for _ in range(size)]
arrow_keys = {(0, 1): '→', (0, -1): '←', (-1, 0): '↑', (1, 0): '↓'}

for k in range(len(path) - 1):
    r1, c1 = path[k]
    r2, c2 = path[k + 1]
    dx, dy = r2 - r1, c2 - c1
    arrow_map[r1][c1] = arrow_keys[(dx, dy)]

arrow_map[goal_pos[0]][goal_pos[1]] = 'F'  # Mark the goal with 'F'

print("\nMap with the Traveled Route:")
for row in arrow_map:
    print(" ".join(row))
```
### **Brief Description**
This code simulates an exploring robot on a 5x5 terrain matrix. The robot starts at position (0,0) and must find a way to the goal in the bottom-right corner while avoiding randomly placed obstacles (X). It moves in four possible directions (R: right, D: down, L: left, U: up), and if blocked, it rotates clockwise until it finds an open path.

The traveled path is recorded and displayed using *, while another map represents the movement using arrows (→, ↓, ↑, ←). If the robot finds no way out, it prints "No route available." Otherwise, it confirms "The robot reached the destination."

## **Problem 7**
A store wants to manage its product inventory. To do this, you must implement a system in Python that allows:  
- Creating products, each with a name, price, and stock quantity.  
- Updating the stock quantity when products are sold.  
- Displaying product information with its availability.  
- Calculating the total inventory value (price × quantity of each product).  

### **Explained Solution**
This code implements a **product inventory system** using **Object-Oriented Programming (OOP)**. The program allows adding, selling, querying, and displaying products, ensuring that the inventory is properly managed.

---

## **Library Import**
```python
import random
```
import random: The random library is imported, which will be used to generate random IDs for the products in the inventory.

#### **Class Producto: Representation of a Product**
This class defines the basic structure of a product, including ID, name, price, and stock quantity.

```python
class Producto:
    def __init__(self, id_producto, nombre, precio, cantidad):
        """Initializes a product with its ID, name, price, and stock quantity."""
        self.id = id_producto
        self.nombre = nombre
        self.precio = precio
        self.stock = cantidad  
```
- self.id: Stores the unique identifier of the product.
- self.nombre: Stores the product's name.
- self.precio: Represents the unit price of the product.
- self.stock: Indicates the available quantity of the product in inventory.

#### **Method vender(): Reduces Stock When Selling a Product**
```python
def vender(self, cantidad):
    """Reduces the stock quantity when a sale is made."""
    if cantidad > self.stock:
        return False  # Not enough stock for the sale
    self.stock -= cantidad  # The sold quantity is deducted from the stock
    return True  # Indicates that the sale was successful
```
- Verifies if there is enough stock before proceeding with the sale.
- If sufficient stock is available, it is reduced, and True is returned.
- If there is not enough stock, it returns False.

#### **Method info(): Displays Product Information**
```python
def info(self):
    """Returns a string with the product's information."""
    estado = "Available" if self.stock > 0 else "Not available"
    return f"ID: {self.id} | {self.nombre} | ${self.precio:.2f} | Stock: {self.stock} | Status: {estado}"
```
- Returns a string with the product details.
- If the stock is greater than 0, the status will be "Available"; otherwise, "Not available".

#### **Class Inventario: Product Management**
This class handles adding, selling, querying, and displaying products in the inventory.

```python
class Inventario:
    def __init__(self):
        """Initializes an empty dictionary to store products."""
        self.productos = {}
```
- self.productos: A dictionary that stores products in the format {ID: Producto}.

#### **Method agregar_producto(): Adds Products to Inventory**
```python
def agregar_producto(self, nombre, precio, cantidad):
    """Adds a product with a randomly generated unique ID."""
    id_producto = random.randint(1000, 9999)  # Generates a unique random ID
    while id_producto in self.productos:
        id_producto = random.randint(1000, 9999)

    self.productos[id_producto] = Producto(id_producto, nombre, precio, cantidad)
    print(f"\nProduct added: {nombre} (ID: {id_producto})")
```
- Generates a random ID between 1000 and 9999.
- Ensures the ID is unique before adding it to the dictionary.
- Creates an instance of Producto and stores it in self.productos.

#### **Method vender_producto(): Reduces Stock or Removes Products**
```python
def vender_producto(self, id_producto, cantidad):
    """Sells a product if it exists in inventory and there is enough stock available."""
    producto = self.productos.get(id_producto)
    if not producto:
        print("\nNo product found with this ID.")
    elif producto.vender(cantidad):
        print(f"\nSale completed: {cantidad} units of '{producto.nombre}'.")
        if producto.stock == 0:
            print(f"'{producto.nombre}' is out of stock and has been removed.")
            del self.productos[id_producto]
    else:
        print(f"\nInsufficient stock. {producto.stock} units remaining.")
```
- Searches for the product in inventory.
- If it exists, it attempts to sell the specified quantity.
- If the stock reaches 0, the product is removed from inventory.

#### **Class Tienda: Menu Management and User Interaction**
This class is responsible for managing user interaction through a menu.

```python
class Tienda:
    def __init__(self):
        """Initializes the store by creating an instance of the Inventory class and executing the main menu."""
        self.inventario = Inventario()
        self.ejecutar()
```
- Creates an instance of Inventario.
- Executes the menu upon initialization.

#### **Method ejecutar(): Main Menu**
```python
def ejecutar(self):
    """Runs the menu options and allows user interaction."""
    opciones = {
        "1": self.agregar,
        "2": self.vender,
        "3": self.consultar_producto_agregado,
        "4": self.mostrar,
        "5": self.valor_total,
        "6": exit
    }
    while True:
        print("\nMenu:")
        print(" 1. Add product")
        print(" 2. Sell product")
        print(" 3. Query product (ID, name, price, and stock)")
        print(" 4. Show all products")
        print(" 5. View total inventory value")
        print(" 6. Exit")

        opcion = input("Select an option: ")
        opciones.get(opcion, self.error)()
```
- Displays the available options in the system.
- Associates each option with a method using a dictionary.
- Executes the corresponding function based on the selected option.

#### **Program Execution**
```python
# Run the program
Tienda()
```
- Starts the program by creating an instance of Tienda.
- The menu is automatically activated, allowing user interaction.

# Libraries Used

In this laboratory, the random library was used, which allows generating random numbers. This was specifically used to assign unique product IDs dynamically.

# References
- Downey, A. (2015). Think Python: How to Think Like a Computer Scientist. O'Reilly Media.
- Shaw, Z. (2019). Learn Python 3 the Hard Way. Addison-Wesley.
- Zelle, J. (2016). Python Programming: An Introduction to Computer Science. Franklin, Beedle & Associates.
- Martelli, A., Ravenscroft, A., & Holden, S. (2020). Python in a Nutshell. O’Reilly Media.
- Sebesta, R. (2019). Concepts of Programming Languages. Pearson.
- Stroustrup, B. (2018). The C++ Programming Language. Addison-Wesley.


