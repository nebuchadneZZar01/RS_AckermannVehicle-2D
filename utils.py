
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

def readGraphFiles(nodes_file, links_file):
    nodes = readNodesFile(nodes_file)
    links = readLinksFile(links_file)

    return nodes, links 

def pixel2meter(p_x, p_y, W=10, H=10):
    m_x = (30 + W + p_x)/1280
    m_y = (600 - H - p_y)/1280

    return m_x, m_y