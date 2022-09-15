from utils import *
from random import seed
from random import randint

INF = 99999999      # infinite distance

class Block:
    def __init__(self, in_node, x_P_pos, y_P_pos, x_M_pos, y_M_pos, color):
        self.in_node = in_node          # specifies in which node the block is located
        self.x_P_pos = x_P_pos          # x pixel position
        self.y_P_pos = y_P_pos          # y pixel position
        self.x_M_pos = x_M_pos          # x meter position
        self.y_M_pos = y_M_pos          # y meter position
        self.color = color              # color of the block
        self.held = False               # specifies if the block is held by the robot

    def setHeld(self):
        if self.held == False:
            self.held = True
    
    def information(self):
        print(self.in_node, self.x_pos, self.y_pos, self.color)

    def getMPos(self):
        return (self.x_M_pos, self.y_M_pos)

    def getColor(self):
        if self.color == 0: return 'red'
        elif self.color == 1: return 'green'
        elif self.color == 2: return 'blue'


class Tower:
    def __init__(self, x_P_pos, y_P_pos, x_M_pos, y_M_pos, color):
        self.x_P_pos = x_P_pos          # x pixel position
        self.y_P_pos = y_P_pos          # y pixel position
        self.x_M_pos = x_M_pos          # x meter position
        self.y_M_pos = y_M_pos          # y meter position
        self.color = color              # color of the tower
        self.n_blocks = 0               # num of blocks in the tower

    def addBlock(self):
        self.n_blocks += 1

    def getColor(self):
        if self.color == 0: return 'red'
        elif self.color == 1: return 'green'
        elif self.color == 2: return 'blue'

    def getMPos(self):
        return (self.x_M_pos, self.y_M_pos)


class Obstacle:
    def __init__(self, x_P_pos, y_P_pos, x_M_pos, y_M_pos):
        self.x_P_pos = x_P_pos          # x pixel position
        self.y_P_pos = y_P_pos          # y pixel position
        self.x_M_pos = x_M_pos          # x meter position
        self.y_M_pos = y_M_pos          # y meter position

    def getPPos(self):
        return (self.x_P_pos, self.x_M_pos)


# World in which the robot is located, and which contains
# blocks to gather and obstacles to avoid
class World:
    def __init__(self):
        seed(1)
        self.nodes = readNodesFile('nodes.txt')
        self.edges = readLinksFile('links.txt')
        self.obstacles = readObstaclesFile('obstacles.txt')

        self.blocks = []
        self.generated_blocks = False

        self.red_tower = None
        self.green_tower = None
        self.blue_tower = None

        for node in self.nodes:
            (x_M, y_M) = pixel2meter(node[1],node[2],30,600,1130)
            if len(node[0]) == 2:
                if node[0][1] == 'r': self.red_tower = Tower(node[1],node[2],x_M,y_M,0)
                if node[0][1] == 'g': self.green_tower = Tower(node[1],node[2],x_M,y_M,1)
                if node[0][1] == 'b': self.blue_tower = Tower(node[1],node[2],x_M,y_M,2)

    # generates, in random positions (from the ones choosed) 
    # and in random colors, n_blocks blocks
    def generateBlocks(self, n_blocks):
        self.blocks.clear()

        choosen_blocks = []

        while len(choosen_blocks) < n_blocks:
            row = randint(0,len(self.nodes)-1)
            if row not in choosen_blocks:
                if self.nodes[row][3] == True:
                    choosen_blocks.append(row)
                    c = randint(0,2)
                    in_n = self.nodes[row][0]
                    (x_M, y_M) = pixel2meter(self.nodes[row][1],self.nodes[row][2],30,600,1130)
                    block = Block(in_n,self.nodes[row][1],self.nodes[row][2],x_M,y_M,c)
                    self.blocks.append(block)

                    if self.generated_blocks == False:
                        self.generated_blocks = True
        
        print("[WORLD] : There are " + str(len(self.blocks)) + " blocks")

    # calculates distance between the robot and the closest block
    def closestBlockDistance(self, robot):
        min_dist = INF
        for block in self.blocks:
            dist = euclideanDistance(robot, block.getMPos())
            if dist < min_dist:
                min_dist = dist
        print("[WORLD] : Nearest block is at dist", min_dist)

        return min_dist

    # detects closest block color
    def closestBlockColor(self, robot):
        col = None
        min_dist = INF
        for block in self.blocks:
            dist = euclideanDistance(robot, block.getMPos())
            if dist < min_dist:
                min_dist = dist
                col = block.getColor()
        if col: 
            print("[WORLD] : Nearest block color is", col)

        return col

    # returns the number of blocks of a certain color
    def getBlocksNumberByColor(self, color):
        count = 0
        for b in self.blocks:
            if b.color == color: count += 1

        return count
    
    # returns the total number of blocks
    def getBlocksNumber(self):
        return len(self.blocks)

    def removeBlock(self, block):
        self.blocks.remove(block)

