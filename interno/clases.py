""" import queue
# from utils import Controlador

class Celda:
    id = 0
    tipo = ""
    nombre = ""
    arriba = False
    derecha = False
    abajo = False
    izquierda = False
    def __init__(self, id, tipo, datos):
        self.id = id
        self.tipo = tipo
        self.nombre = datos["nombre"]
        self.arriba = datos["arriba"]
        self.derecha = datos["derecha"]
        self.abajo = datos["abajo"]
        self.izquierda = datos["izquierda"]
    def __str__(self):
        return f"[{self.id}] {self.nombre} -> arriba-{self.arriba} ; derecha-{self.derecha} ; abajo-{self.abajo} ; izquierda-{self.izquierda}"

class Casilla:
    id = ""
    row = 0
    col = 0
    def __init__(self, id, row, col):
        self.id = id
        self.row = row  
        self.col = col

class Pedido:
    id = ""
    inicio = Casilla("", 0, 0)
    destino = Casilla("", 0, 0)
    def __init__(self, id, inicio, destino):
        self.id = id
        self.inicio = inicio  
        self.destino = destino
        
class Robot:
    # Atributos
    name = "robotito"
    estado = "parado"   # parado, trabajando
    orientacion = "derecha" # arriba, derecha, abajo, izquierda
    casilla_actual = 30 # abajo a la izquierda
    control = None
    # Robot
    # robot = EV3Brick()
    # left_motor = Motor(Port.A)  # motor rueda izquierda
    # right_motor = Motor(Port.D) # motor rueda derecha
    drive = None
    # Constructor
    # def __init__(self):
    #     self.control = Controlador()
    #     self.drive = DriveBase(self.left_motor, 
    #                            self.right_motor, 
    #                            wheel_diameter=self.control.CONFIG["parametros"]["wheel_diameter"], 
    #                            axle_track=self.control.CONFIG["parametros"]["axle_track"]
    #                            )
    
    
        

 """