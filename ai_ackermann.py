from phidias.Types import *
from phidias.Main import *
from phidias.Lib import *
from phidias.Agent import *
from utils import *

# --- GRAPH INFORMATIONS ---

# Nodes where there may be blocks
class blockSlot(Belief): pass
class block(SingletonBelief): pass

# Block Held
class heldBlock(SingletonBelief): pass

# Information about towers: node, color and current number of blocks
class towerColor(Belief): pass

# Graph edges
class edge(Belief): pass


# --- NAVIGATION ---

# Minimum path generation
class selected(SingletonBelief): pass
class path(Procedure): pass
class select_min(Procedure): pass
class show_min(Procedure): pass

# Prevents cyclic paths
class nodeNotInPath(ActiveBelief):
    def evaluate(self, P, Node):
        return (Node() not in P())

# Graph navigation believes
class robotNode(SingletonBelief): pass
class targetNode(SingletonBelief): pass
class targetIntermediateNode(SingletonBelief): pass
class targetReached(SingletonBelief):pass
class selected_path(SingletonBelief): pass
class not_navigating(SingletonBelief): pass

# Graph navigation procedures
class follow_path(Procedure): pass
class generate_and_follow_min_path(Procedure): pass
class go_to_tower(Procedure): pass
class go_to_start(Procedure): pass

# Check if robot is close to the node
class closeRobotNode(Goal): pass

# Procedure to scan the world, take the blocks and
# put them in the towers
class resolve(Procedure): pass 

# Block nodes not checked
class slotNotChecked(Belief): pass

# Uncheck blocks slot when generating new blocks
# or scanning is ended
class restoreSlots(Procedure): pass

# Remove tower blocks from knowledge base
# when resetting towers
class remove_towers_blocks(Procedure): pass


# --- COMMUNICATION ---

# Procedures that talk to GUI server
class send_heldBlock(Procedure) : pass
class send_releaseBlock(Procedure) : pass
class sense(Procedure) : pass
class generate(Procedure) : pass
class go_node(Procedure) : pass
class go(Procedure) : pass
class reset_towers(Procedure): pass

# Server beliefs
class go_to_node(Belief): pass
class go_to(Belief): pass
class send_held_block(Belief): pass
class sense_color(Belief): pass
class sense_distance(Belief): pass
class generate_blocks(Belief): pass
class releaseBlockToTower(Belief): pass
class resetTowers(Belief):pass
class change_control_type(Belief):pass

# Reactors
class target_got(Reactor): pass
class distance(Reactor): pass
class color(Reactor): pass

def_vars('Src', 'Dest', 'Next', 'Cost', 'P', 'Total', 'CurrentMin', 'CurrentMinCost','Node','robot','pathLength','N','currentTarget','X','Z','C','_A','D')

class main(Agent):
    def main(self):
        go(X, Z) >> [ +go_to(X, Z)[{'to': 'robot@127.0.0.1:6566'}] ]

        go_node(Node) >> [
            show_line("[ROBOT COMMUNICATION] : sending go node request to node ", Node), 
            +go_to_node(Node)[{'to': 'robot@127.0.0.1:6566'}] 
        ]
        
        +targetReached()[{'from': _A}] >> [ show_line("Target reached") ]
          
        go_to_start() / robotNode(robot) >> [
            show_line("[ROBOT] : Returning to START point"),
            path(robot,"START",[]),
            "N = 1",
            +targetReached(robot),
            follow_path(robot)
        ]

        generate_and_follow_min_path(Node, P, N, pathLength) / (blockSlot(Node) & robotNode(robot)) >> [
            path(robot, Node, P),
            "N = 1",
            +targetReached(robot),
            follow_path(robot)
        ]

        generate_and_follow_min_path(Node, P, N, pathLength) / (towerColor(Node, C, X) & robotNode(robot)) >> [
            path(robot, Node, P),
            "N = 1",
            +targetReached(robot),
            follow_path(robot)
        ]

        follow_path(currentTarget) / (selected_path(P, pathLength, N) & eq(pathLength, N)) >> [ 
            show_line("[ROBOT] : target reached"),
            "N = 1"
        ]

        follow_path(currentTarget) / (targetReached(Node) & selected_path(P,pathLength,N) ) >> [ 
            "currentTarget = P[N]",
            "N = N+1",
            +selected_path(P, pathLength, N),
            +targetIntermediateNode(currentTarget),
            -targetReached(Node),
            go_node(currentTarget)
        ]

        follow_path(currentTarget) >> []

        path(Src, Dest, P) >> \
        [
            path(P, 0, Src, Dest),
            show_min(P)
        ]

        path(P, Total, Dest, Dest) >> \
        [ 
            "P.append(Dest)", 
            +selected(P, Total)
        ]

        path(P, Total, Node,  Dest) / (edge(Node, Dest, Cost) & nodeNotInPath(P, Dest))  >> \
        [
            "P = P.copy()",
            "P.append(Node)",
            "Total = Total + Cost",
            select_min(P, Total, Dest, Dest)
        ]

        path(P, Total, Src,  Dest)['all'] / (edge(Src, Next, Cost) & nodeNotInPath(P, Next))  >> \
        [
            "P = P.copy()",
            "P.append(Src)",
            "Total = Total + Cost",
            select_min(P, Total, Next, Dest)
        ]

        select_min(P, Total, Next, Dest) >> \
        [
            path(P, Total, Next, Dest)
        ]

        show_min(P) / selected(CurrentMin, CurrentMinCost) >> \
        [
            show_line("[ROBOT] : Minimum Cost Path ", CurrentMin, ", cost ", CurrentMinCost),
            "pathLength = len(CurrentMin)",
            +selected_path(CurrentMin, pathLength, 1),
            -selected(CurrentMin,CurrentMinCost),
        ]
        
        +target_got()[{'from': _A}] / (targetIntermediateNode(Node) & targetNode(Node) & heldBlock(X, C) & towerColor(Node, C, N) ) >> \
        [
            show_line('[ROBOT] : Reached Tower ', Node),
            +targetReached(Node),
            +robotNode(Node),
            +not_navigating("1"),
            -heldBlock(X, C),
            send_releaseBlock(),
            resolve()
        ]

        +target_got()[{'from': _A}] / (targetIntermediateNode(Node) & eq(Node, "START")) >> \
        [
            +targetReached(Node),
            +robotNode(Node),
            restoreSlots()
        ]

        +target_got()[{'from': _A}] / (targetIntermediateNode(Node) & targetNode(Node)) >> \
        [
            show_line('[ROBOT] : Reached Node ', Node),
            +targetReached(Node),
            +robotNode(Node),
            sense()
        ]

        +target_got()[{'from': _A}] / (targetIntermediateNode(X)) >> \
        [
            show_line('[ROBOT] : Reached intermediate Node ', X),
            +targetReached(X),
            +robotNode(X),
            sense()
        ]

        closeRobotNode(Node, D) << (robotNode(Node) & lt(D, 0.5) & slotNotChecked(Node))

        +distance(D)[{'from':_A}] / closeRobotNode(Node, D) >> [ 
            show_line("[ROBOT] : Block found in slot ", Node),
            +block(Node)
        ]

        +distance(D)[{'from':_A}] / (targetIntermediateNode(Node) & targetNode(Node)) >> [ 
            show_line("[ROBOT COMMUNICATION] : Received ", D, " from ROBOT"),
            +not_navigating("1"),
            -slotNotChecked(Node), 
            resolve()
        ]

        +distance(D)[{'from':_A}] / robotNode(robot) >> [
            +not_navigating("1"), 
            -slotNotChecked(robot), 
            follow_path(robot)
        ]

        +color(C)[{'from':_A}] / (block(X) & towerColor(Node, C, N) & geq(N, 3)) >> [ 
            show_line("[ROBOT] : Tower ", C, " full, cannot pick block sampled in slot ", X),
            -block(X),
            -slotNotChecked(X),
            +not_navigating("1"),
            resolve()
        ]

        +color(C)[{'from':_A}] / (block(X) & towerColor(Node, C, N) & lt(N, 4)) >> [ 
            show_line("[ROBOT] : Color ", C, " sampled in slot ", X),
            -block(X),
            +heldBlock(X, C),
            -slotNotChecked(X),
            send_heldBlock(X),
            +targetNode(Node),
            go_to_tower(Node)
        ]

        go_to_tower(Node) / (heldBlock(X, C) & towerColor(Node, C, Z))  >> [
            -towerColor(Node, C, Z),
            "Z = Z + 1", 
            +towerColor(Node, C, Z),
            generate_and_follow_min_path(Node, [], 0, 0), 
        ]

        
        # RESOLVE PROCEDURES

        resolve() / (blockSlot(Node) & slotNotChecked(Node)) >> [ resolve(Node) ]

        resolve() / heldBlock(X, C) >> [ ]

        resolve() >> [
            show_line("[ROBOT]: Scanning has completed"),
            go_to_start()
        ]

        resolve(Node) / (slotNotChecked(Node) & blockSlot(Node) & robotNode(robot) & not_navigating(C)) >> [ 
            +targetNode(Node),
            -not_navigating(C),
            generate_and_follow_min_path(Node, [], 0 ,0)
        ]

        restoreSlots()['all'] / blockSlot(Node) >> [ +slotNotChecked(Node) ]

        send_heldBlock(Node) >> [ +send_held_block(Node)[{'to': 'robot@127.0.0.1:6566'}] ]

        send_releaseBlock() >> [ +releaseBlockToTower()[{'to': 'robot@127.0.0.1:6566'}] ]

        sense() / heldBlock(X, C) >> [ follow_path(X) ]

        sense() >> [ 
            +sense_distance()[{'to': 'robot@127.0.0.1:6566'}],
            +sense_color()[{'to': 'robot@127.0.0.1:6566'}] 
        ]

        generate() >> [ 
            show_line("[ROBOT COMMUNICATION] : richiesta generazione 9 blocchi"),
            +generate_blocks(9)[{'to': 'robot@127.0.0.1:6566'}] ,
            restoreSlots()
        ]

        generate(N) / gt(N, 9) >> [ show_line("[ROBOT] : cannot generate more than 9 blocks") ]
        
        generate(N) >> [
            show_line("[ROBOT COMMUNICATION] : richiesta generazione ", N," blocchi"), 
            +generate_blocks(N)[{'to': 'robot@127.0.0.1:6566'}], 
            restoreSlots()
        ]

nodes = readNodesFile('nodes.txt')
edges = readLinksFile('links.txt')

ag = main()
ag.start()

for n in nodes:
    if len(n[0]) == 1:
        ag.assert_belief(slotNotChecked(n[0]))

for n in nodes:
    if n[3] == True: ag.assert_belief(blockSlot(n[0]))

ag.assert_belief(robotNode('START'))
ag.assert_belief(not_navigating("1"))

for e in edges:
    ag.assert_belief(edge(e[0],e[1],e[2]))
    ag.assert_belief(edge(e[1],e[0],e[2]))

ag.assert_belief(towerColor('Tr','red',0))
ag.assert_belief(towerColor('Tg','green',0))
ag.assert_belief(towerColor('Tb','blue',0))

PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())