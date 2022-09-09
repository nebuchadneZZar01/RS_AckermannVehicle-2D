class Block:
    def __init__(self, in_node, x_P_pos, y_P_pos, x_M_pos, y_M_pos, color):
        self.in_node = in_node
        self.x_P_pos = x_P_pos
        self.y_P_pos = y_P_pos
        self.x_M_pos = x_M_pos
        self.y_M_pos = y_M_pos
        self.color = color
        self.held = False

    def setHeld(self):
        if self.held == False:
            self.held = True
    
    def information(self):
        print(self.in_node, self.x_pos, self.y_pos, self.color)

class Tower:
    def __init__(self, color):
        self.color = color
        self.n_blocks = 0

    def addBlock(self):
        self.n_blocks += 1
