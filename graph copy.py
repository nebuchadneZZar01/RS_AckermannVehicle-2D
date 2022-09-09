from phidias.Types import *
from phidias.Main import *
from phidias.Lib import *
from phidias.Agent import *
from utils import *

# --- GRAPH ---

class node(Belief): pass
class link(Belief): pass
class path(Procedure): pass

# --- ROBOT BEHAVIOR ---

class go_to(Belief): pass
class target_reached(Reactor): pass
class node_reached(Reactor): pass
class go(Procedure): pass

def_vars('Source', 'Destination', 'Next', 'Cost', 'P', 'Total', 'CurrentMin', 'CurrentMinCost', 'X', 'Y', 'F')

class main(Agent):
    def main(self):
        go(X) >> [ +go_to(X)[{'to': 'robot@127.0.0.1:6566'}] ]
        
        +target_reached()[{'from': F}] >> [ show_line("Target reached") ]
          
        path(Source, Destination) / link(Source, Next, Cost) >> [ 
            show_line("Next node is ", Source), 
            go(Source), 
            path(Source, Destination, Next, Cost)
        ]

        path(Destination, Destination) >> [ show_line("End node is ", Destination) ]

        path(Source, Destination, CurrentMin, CurrentMinCost) / (link(Source, Next, Cost) & lt(Cost, CurrentMinCost)) >> [ path(Source, Destination, Next, Cost) ]

        path(Source, Destination, CurrentMin, CurrentMinCost) >>  [
            +node_reached()[{'from': F}] >>  [
                show_line("Node reached"),
                path(CurrentMin, Destination)
            ]
        ]

edges = readLinksFile('links.txt')

ag = main()
ag.start()

for e in edges:
    ag.assert_belief(link(e[0],e[1],e[2]))

PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())