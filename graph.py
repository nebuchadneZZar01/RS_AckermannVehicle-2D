from phidias.Types import *
from phidias.Main import *
from phidias.Lib import *
from phidias.Agent import *

# --- GRAPH ---

class node(Belief): pass
class link(Belief): pass
class path(Procedure): pass

# --- ROBOT BEHAVIOR ---

class go_to(Belief): pass
class target_reached(Reactor): pass
class go(Procedure): pass

def_vars('Source', 'Destination', 'Next', 'Cost', 'P', 'Total', 'CurrentMin', 'CurrentMinCost', 'X', 'Y', 'F')

path(Source, Destination) / link(Source, Next, Cost) >> [ show_line("Next node is ", Source), path(Source, Destination, Next, Cost)]
path(Destination, Destination) >> [ show_line("End node is ", Destination) ]

path(Source, Destination, CurrentMin, CurrentMinCost) / (link(Source, Next, Cost) & lt(Cost, CurrentMinCost)) >> [ path(Source, Destination, Next, Cost) ]

path(Source, Destination, CurrentMin, CurrentMinCost) >>  [ path(CurrentMin, Destination) ]

class main(Agent):
    def main(self):
        go(X,Y) >> [ +go_to(X,Y)[{'to': 'robot@127.0.0.1:6566'}] ]
        +target_reached()[{'from':F}] >> [ show_line("Target reached") ]

PHIDIAS.assert_belief(link('START', 'A', 2))
PHIDIAS.assert_belief(link('START', 'D', 2))

PHIDIAS.assert_belief(link('A', 'B', 5))
PHIDIAS.assert_belief(link('A', 'D', 2))

PHIDIAS.assert_belief(link('B', 'C', 5))
PHIDIAS.assert_belief(link('B', 'F', 3))

PHIDIAS.assert_belief(link('C', 'G', 3))

PHIDIAS.assert_belief(link('D', 'E', 2))

PHIDIAS.assert_belief(link('E', 'F', 4))
PHIDIAS.assert_belief(link('E', 'H', 4))
PHIDIAS.assert_belief(link('E', 'I', 3))

PHIDIAS.assert_belief(link('F', 'I', 3))
PHIDIAS.assert_belief(link('F', 'J', 2))
PHIDIAS.assert_belief(link('F', 'K', 5))
PHIDIAS.assert_belief(link('F', 'G', 4))

PHIDIAS.assert_belief(link('G', 'L', 2))

PHIDIAS.assert_belief(link('H', 'I', 3))

PHIDIAS.assert_belief(link('I', 'F', 3))

PHIDIAS.assert_belief(link('J', 'N', 3))
PHIDIAS.assert_belief(link('J', 'K', 2))

PHIDIAS.assert_belief(link('K', 'N', 5))
PHIDIAS.assert_belief(link('K', 'L', 4))

PHIDIAS.assert_belief(link('L', 'M', 3))

PHIDIAS.assert_belief(link('M', 'END', 4))

PHIDIAS.assert_belief(link('N', 'END', 7))

ag = main()
ag.start()

PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())