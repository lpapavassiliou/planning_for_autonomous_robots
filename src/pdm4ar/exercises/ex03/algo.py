from abc import ABC, abstractmethod
from cmath import inf
from dataclasses import dataclass
from tempfile import TemporaryDirectory
from numpy import Inf
from osmnx.distance import great_circle_vec
import math
from pdm4ar.exercises.ex02.structures import X, Path
from pdm4ar.exercises.ex03.structures import WeightedGraph, TravelSpeed


@dataclass
class InformedGraphSearch(ABC):
    graph: WeightedGraph

    @abstractmethod
    def path(self, start: X, goal: X) -> Path:
        # need to introduce weights!
        pass

@dataclass
class UniformCostSearch(InformedGraphSearch):

    def path(self, start: X, goal: X) -> Path:
        
        Q = {start: 0}
        costToReach = {}
        for node in self.graph.adj_list:
            costToReach[node] = inf
        costToReach[start]=0

        Parent = {start:None}
        path = []
        #V = []

        goalReached = False
        while len(Q) != 0 and goalReached == False:

            #print('Entered in loop...')
            #print('Q is:')
            #print(Q)
            chosenNode = min(Q, key=Q.get)
            #V.append(chosenNode)
            pop = Q.pop(chosenNode)
            #print('Chosen node ' + str(chosenNode))

            if chosenNode == goal:
                #print('Goal founded!!')
                goalReached = True
                i = chosenNode
                while Parent[i] != None:
                    path.insert(0, i)
                    i = Parent[i]
                path.insert(0, start)  # start node does not enter in the while loop

            else:
                for reachable in self.graph.adj_list[chosenNode]:
                    #print('current reachable: '+ str(reachable))
                    #print(str(self.graph.get_weight(chosenNode,reachable)))
                    tempCostToReach = costToReach[chosenNode] + self.graph.get_weight(chosenNode,reachable)
                    if tempCostToReach < costToReach[reachable]:
                        costToReach[reachable] = tempCostToReach
                        Parent[reachable] = chosenNode
                        Q[reachable] = tempCostToReach
                        #print('cost updated')
                        #print('new queue is')
                        #print(Q)
        '''
        if not goalReached:
            print('GOAL NOT FOUNDED')
            print('V is:')
            print(V)
            for node in self.graph.adj_list:
                #print('looking for node...' + str(node))
                founded = False
                for curr_node in V:
                    if curr_node == node:
                        founded = True
                if founded == False:
                    print('GRAPH NOT FULLY EXPLORED \nxxxxxxxxxx\nxxxxxxxxxx\nxxxxxxxxxx\n\n\n\n')
                    print('missing node ' + str(node))
        '''
        #print('UNIFORMED SEARCH COMPLETED')
        return path
        

@dataclass
class Astar(InformedGraphSearch):
 
    def heuristic(self, u: X, v: X) -> float:
        coord_u = self.graph.get_node_coordinates(u)
        coord_v = self.graph.get_node_coordinates(v)
        return (math.sqrt((coord_u[0]-coord_v[0])**2 + (coord_u[1]-coord_v[1])**2))
    
    '''
    def fake_heuristic(self, x:X, v:X) -> float:
        if x == 0:
            return 12
        if x == 1:
            return 10
        if x == 2:
            return 6
        if x == 3:
            return 6
        if x == 4:
            return 9
        if x == 5:
            return 5
        if x == 6:
            return 4
        if x == 7:
            return 8
        if x == 8:
            return 8
        if x == 'E':
            return 0
        else: return 0
    '''

    def path(self, start: X, goal: X) -> Path:
        #print('Path in esecuzione')
        Q = {start: 0}
        costToReach = {}
        for node in self.graph.adj_list:
            costToReach[node] = inf
        costToReach[start]=0

        Parent = {start:None}
        path = []
        #print('Inizializzazione compiuta')
        #V = []

        goalReached = False
        while len(Q) != 0 and goalReached == False:

            #print('Entered in loop...')
            #print('Q is:')
            #print(Q)
            chosenNode = min(Q, key=Q.get)
            pop = Q.pop(chosenNode)
            #print('Chosen node ' + str(chosenNode))

            if chosenNode == goal:
                #print('Goal founded!!')
                goalReached = True
                i = chosenNode
                while Parent[i] != None:
                    path.insert(0, i)
                    i = Parent[i]
                path.insert(0, start)  # start node does not enter in the while loop

            else:
                for reachable in self.graph.adj_list[chosenNode]:
                    #print('current reachable: '+ str(reachable))
                    #print(str(self.graph.get_weight(chosenNode,reachable)))
                    tempCostToReach = costToReach[chosenNode] + self.graph.get_weight(chosenNode,reachable)
                    if tempCostToReach < costToReach[reachable]:
                        costToReach[reachable] = tempCostToReach
                        Parent[reachable] = chosenNode
                        #print('Heuristic for node ' + str(reachable)+' is:')
                        #print(self.heuristic(reachable,goal))
                        Q[reachable] = tempCostToReach + self.heuristic(reachable,goal)
                        #print('cost updated')
                        #print('new queue is')
                        #print(Q)
        #print('A* SEARCH COMPLETED')
        return path


def compute_path_cost(wG: WeightedGraph, path: Path):
    """A utility function to compute the cumulative cost along a path"""
    if not path:
        return float("inf")
    total: float = 0
    for i in range(1, len(path)):
        inc = wG.get_weight(path[i - 1], path[i])
        total += inc
    return total


    '''
    adj_list = {'s':['a','b'], 'a':['c','d'], 'b':['g'], 'c':['s','d'], 'd':['b','g'], 'g':[]
    }
    weights = { ('s','a'):2, ('s','b'):5, ('d','g'):2, ('c','d'):3, ('c','s'):1, ('d','b'):1, ('b','g'):2, ('a','c'):2, ('a','d'):4
    }
    '''
'''
class main():

    print('Main in esecuzione')
    adj_list = {0:[1,2,3], 1:[4,5], 2:[5], 3:[5,7], 4:[8], 5:[6], 6:['E'], 7:[6], 8:[3], 'E':[4]}
    weights = {(0,1):1, (0,2):2, (0,3):4, (1,4):5, (1,5):4, (2,5):3, (3,5):1, (4,8):2, (5,6):2, (7,6):5, (6,'E'):5, (3,7):3, (8,3):-4}
    
    _G = {}
    graph = WeightedGraph(adj_list, weights, _G)
    search = Astar(graph)
    path = search.path(0, 'E')
    print(path)
'''

