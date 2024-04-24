import os
import json
from queue import Queue
import networkx as nwx
# "mapa": 02 02 00 01 05 - 03 07 05 00 02 - 00 04 11 09 06 - 01 10 03 10 00 - 00 02 00 08 01 - 01 10 01 10 00 - 01 06 01 07 01

# ----------------------------------------------------------------------------------------------------------------------------------------------------

class Controlador:
    # Configurador: parámetros...
    CONFIG = None
    # Mapa
    mapa_sin_procesar = None
    mapa = None
    def __init__(self):
        # Cargar el archivo de configuración
        with open('interno/config.json', 'r') as f:
            self.CONFIG = json.load(f)
        # Crear la cola de pedidos
        self.__cola_pedidos = Queue()
        # Cargar el mapa
        # mapa_sin_procesar = cargar()
        # Procesar el mapa
        mapa = __procesar_mapa()
        
    # Cola de pedidos
    # Funciones de gestión de la cola
    def comprobar_cola_vacia(self):
        return self.__cola_pedidos.empty()
    def comprobar_cola_no_vacia(self):
        return (not self.__cola_pedidos.empty())
    def siguiente_cola(self):
        if (self.__cola_pedidos.empty()): return None
        return __cola_pedidos.get_nowait()
    def meter_en_cola(self, elemento: str):
        self.__cola_pedidos.put_nowait(elemento)
    def get_cola(self):
        return self.__cola_pedidos.queue
    
    # Funciones de mapa
    def __procesar_mapa(self)
        # "mapa": 02 02 00 01 05 - 03 07 05 00 02 - 00 04 11 09 06 - 01 10 03 10 00 - 00 02 00 08 01 - 01 10 01 10 00 - 01 06 01 07 01
        mapa_sin_procesar = "0202000105030705000200041109060110031000000200080101100110000106010701"
        grafito = nwx.Graph()

        


# ----------------------------------------------------------------------------------------------------------------------------------------------------

control = Controlador()
print(control.CONFIG["parametros"]["giro"])
print(control.comprobar_cola_vacia())