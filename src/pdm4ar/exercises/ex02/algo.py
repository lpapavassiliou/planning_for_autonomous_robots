from abc import abstractmethod, ABC
from typing import Tuple

from pdm4ar.exercises.ex02.structures import AdjacencyList, X, Path, OpenedNodes


class GraphSearch(ABC):
    @abstractmethod
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        """
        :param graph: The given graph as an adjacency list
        :param start: The initial state (i.e. a node)
        :param goal: The goal state (i.e. a node)
        :return: The path from start to goal as a Sequence of states, None if a path does not exist
        """
        pass


class DepthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        # todo implement here your solution
        numNodes = len(graph)
        Q = [start]
        V = []
        Parent = {start:None}
        path = []
        goalReached = False

        while len(Q) != 0 and goalReached == False:
            chosenNode = Q[0]
            V.append(chosenNode)
            Q = Q[1:]

            if chosenNode == goal:
                goalReached = True
                i = chosenNode
                while Parent[i] != None:
                    path.insert(0, i)
                    i = Parent[i]
                path.insert(0, start)  # start node does not enter in the while loop
            else:
                addingList = []
                for nodeNext in graph.get(chosenNode):

                    nodeNextExplored = False
                    for n in V+Q:
                        if n == nodeNext:
                            nodeNextExplored = True

                    if not nodeNextExplored:
                        addingList.append(nodeNext)
                        Parent[nodeNext] = chosenNode

                addingList.sort()
                Q = addingList+Q

        return path, V


class BreadthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        # todo implement here your solution
        numNodes = len(graph)
        Q = [start]
        V = []
        Parent = {start:None}
        path = []

        goalReached = False
        while len(Q) != 0 and goalReached == False:
            chosenNode = Q[0]
            V.append(chosenNode)
            Q = Q[1:]

            if chosenNode == goal:
                goalReached = True
                i = chosenNode
                while Parent[i] != None:
                    path.insert(0, i)
                    i = Parent[i]
                path.insert(0, start)  # start node does not enter in the while loop

            else:
                addingList = []

                for nodeNext in graph.get(chosenNode):

                    nodeNextExplored = False
                    for n in V+Q:
                        if n == nodeNext:
                            nodeNextExplored = True


                    if not nodeNextExplored:
                        addingList.append(nodeNext)
                        Parent[nodeNext] = chosenNode

                addingList.sort()
                Q = Q + addingList

        return path, V


class IterativeDeepening(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        # todo implement here your solution
        d_max = 0
        goalReached = False
        maxDepthReached = True

        while goalReached == False and maxDepthReached == True:

            Q = [start]
            V = []
            numNodes = len(graph)
            Parent = {start:None}
            path = []
            maxDepthReached = False

            while len(Q) != 0 and not goalReached:

                chosenNode = Q[0]
                V.append(chosenNode)
                Q = Q[1:]
                goalReached = False

                if chosenNode == goal:
                    goalReached = True  # stops while loop
                    i = chosenNode
                    while (Parent[i] != None):
                        path.insert(0, i)
                        i = Parent[i]
                    path.insert(0, start)

                else:
                    # counting parents of chosenNode to determine its depth d
                    # counting starts from 0.
                    d = 0
                    i = chosenNode
                    while Parent[i] != None:
                        d = d + 1
                        i = Parent[i]

                    if d == d_max:
                        maxDepthReached = True

                    if d < d_max and goalReached == False:  # checks if we reached max depth
                        addingList = []

                        for nodeNext in graph.get(chosenNode):

                            nodeNextExplored = False
                            for n in V+Q:
                                if n == nodeNext:
                                    nodeNextExplored = True


                            if not nodeNextExplored:
                                addingList.append(nodeNext)
                                Parent[nodeNext] = chosenNode

                        addingList.sort()
                        Q = addingList + Q
                        
            d_max = d_max + 1

        return path, V