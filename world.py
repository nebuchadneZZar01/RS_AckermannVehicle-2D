from dis import dis
from utils import *
from random import seed
from random import randint
import math

INF = 99999999

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

    def getMPos(self):
        return (self.x_M_pos, self.y_M_pos)

class Tower:
    def __init__(self, color):
        self.color = color
        self.n_blocks = 0

    def addBlock(self):
        self.n_blocks += 1

class World:
    def __init__(self):
        seed(1)
        self.nodes = readNodesFile('nodes.txt')
        self.edges = readLinksFile('links.txt')

        self.blocks = []
        self.towers = []
        self.generated_blocks = False

    def generateBlocks(self):
        self.blocks.clear()

        for n in self.nodes:
            if n[3] == True: 
                c = randint(0,2)
                in_n = n[0]
                (x_M, y_M) = pixel2meter(n[1],n[2])
                block = Block(in_n,n[1],n[2],x_M,y_M,c)
                self.blocks.append(block)

                if self.generated_blocks == False:
                    self.generated_blocks = True

        print("There are " + str(len(self.blocks)) + " blocks")

    def closestBlockDistance(self, robot):
        min_dist = INF
        for block in self.blocks:
            dist = euclideanDistance(robot, block.getMPos())
            if dist < min_dist:
                min_dist = dist
        print("Nearest block is at dist", min_dist)

        return min_dist

    def closestBlockColor(self, robot):
        min_dist = INF
        for block in self.blocks:
            dist = euclideanDistance(robot, block.getMPos())
            if dist < min_dist:
                min_dist = dist
                col = block.color

        return col

    def getBlocksNumberByColor(self, color):
        count = 0
        for b in self.blocks:
            if b.color == color: count += 1

        return count

    def getBlocksNumber(self):
        return len(self.blocks)

