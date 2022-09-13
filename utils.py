
import math
from posixpath import split

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

def readNodesFile(nodes_file):
    with open(nodes_file, 'r') as f:
        nodes = []
        for line in f.readlines()[1::]:
            l = line.rstrip().split(',')
            l[1] = int(l[1])
            l[2] = int(l[2])
            l[3] = str2bool(l[3])
            l = tuple(l)
            nodes.append(l)
    
    return nodes

def readLinksFile(links_file):
    with open(links_file, 'r') as f:
        links = []
        for line in f.readlines()[1::]:
            l = line.rstrip().split(',')
            l[2] = int(l[2])
            l = tuple(l)
            links.append(l)
    
    return links

def readObstaclesFile(obstacles_file):
    with open(obstacles_file, 'r') as f:
        obstacles = []
        for line in f.readlines()[1::]:
            l = line.rstrip().split(',')
            l[0] = int(l[0])
            l[1] = int(l[1])
            l = tuple(l)
            obstacles.append(l)
    
    return obstacles

def readGraphFiles(nodes_file, links_file):
    nodes = readNodesFile(nodes_file)
    links = readLinksFile(links_file)

    return nodes, links 

def pixel2meter(p_x, p_y, W, H, bot_size):
    m_x = (- 30 + p_x)/1130
    m_y = (600 - p_y)/1130

    return m_x, m_y

def euclideanDistance(node_A, node_B):
    return math.sqrt((node_A[0]-node_B[0])**2 + (node_A[1]-node_B[1])**2)