
import math
from posixpath import split

# converts "True"/"False" or "0"/"1" strings into booleans
def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

# to read the file containing the graph's nodes
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

# to read the file containing the graph's edges
def readLinksFile(links_file):
    with open(links_file, 'r') as f:
        links = []
        for line in f.readlines()[1::]:
            l = line.rstrip().split(',')
            l[2] = int(l[2])
            l = tuple(l)
            links.append(l)
    
    return links

# to read all graph files one-shot
def readGraphFiles(nodes_file, links_file):
    nodes = readNodesFile(nodes_file)
    links = readLinksFile(links_file)

    return nodes, links 

# to read the file containing the obstacles
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

# to convert pixel positions into meters
# as the robots supports only meters
def pixel2meter(p_x, p_y, offset_x, offset_y, width):
    m_x = (- offset_x + p_x)/width
    m_y = (offset_y - p_y)/width

    return m_x, m_y

# to calculate the euclidean distance between two nodes
# or the one between the robot current position (node)
# and another one
def euclideanDistance(node_A, node_B):
    return math.sqrt((node_A[0]-node_B[0])**2 + (node_A[1]-node_B[1])**2)