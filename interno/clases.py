import queue

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
        
q = queue.Queue()
print(q.qsize())

print(q.get_nowait())

""" for i in q.queue:
    print(i)
print(q.qsize())
print(type(q.queue)) """

