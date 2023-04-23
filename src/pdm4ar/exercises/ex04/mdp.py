from abc import ABC, abstractmethod
from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from pdm4ar.exercises.ex04.structures import Action, Cell, Policy, State, ValueFunc


class GridMdp:
    def __init__(self, grid: NDArray[np.int], gamma: float = 0.9):
        assert len(grid.shape) == 2, "Map is invalid"
        self.grid = grid
        """The map"""
        self.gamma: float = gamma
        """Discount factor"""

    def manh_distance(self, state1:State, state2:State):
        return np.absolute(state1[0]-state2[0])+np.absolute(state1[1]-state2[1])

    def get_actions(self, state:State):
        if self.out_of_bounds(state):
            return Action.SOUTH
        if self.grid[state] == Cell.GOAL:
            return [Action.STAY]  
        actions = [Action.NORTH, Action.SOUTH, Action.WEST, Action.EAST]
        x = state[0]
        y = state[1]
        if self.out_of_bounds((x+1,y)):
            actions.remove(Action.SOUTH)
        if self.out_of_bounds((x-1,y)):
            actions.remove(Action.NORTH)
        if self.out_of_bounds((x,y+1)):
            actions.remove(Action.EAST)
        if self.out_of_bounds((x,y-1)):
            actions.remove(Action.WEST)
        return actions
    
    def out_of_bounds(self, state:State):
        # returns 
        N = self.grid.shape[0]
        M = self.grid.shape[1]
        if state[0]>= N or state[0] < 0:
            return True
        if state[1]>= M or state[1] < 0:
            return True
        return False

    def get_reachable(self, state:State, action: Action, start:State):
        if action == Action.STAY:
            return [state]
        if self.out_of_bounds(state):
            return [start]
        if self.grid[state] == Cell.GOAL:
            return [state]
        x = state[0]
        y = state[1]
        if self.grid[state] == Cell.SWAMP:
            return [state,(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]

    def get_transition_prob(self, state: State, action: Action, next_state: State) -> float:
        """Returns P(next_state | state, action)"""

        # -------------  OUT OF BOUNDS --------------------------
        # if self.out_of_bounds(state):
        #     print('Probability 1 of going back to start.')
        #     if self.grid[next_state] == Cell.START:
        #         return 1
        #     else:
        #         return 0

        # ---------------read the cell -------------------------

        curr_cell = self.grid[state]

        # ------------------- GOAL -----------------------------
        if curr_cell == Cell.GOAL:
            if action == Action.STAY:
                return 1
            else:
                return 0

        # ------------------- GRASS ----------------------------
        if curr_cell == Cell.START or curr_cell == Cell.GRASS:
            if next_state == state:
                return 0
            if action == Action.NORTH:
                if next_state[0] == state[0]-1 and next_state[1] == state[1]:
                    return 0.75
            if action == Action.SOUTH:
                if next_state[0] == state[0]+1 and next_state[1] == state[1]:
                    return 0.75
            if action == Action.WEST:
                if next_state[0] == state[0] and next_state[1] == state[1]-1:
                    return 0.75
            if action == Action.EAST:
                if next_state[0] == state[0] and next_state[1] == state[1]+1:
                    return 0.75
            
            if self.manh_distance(state, next_state) == 1:
                return 1/12
            return 0
        
        # ------------------- SWAMP  ----------------------------
        if (curr_cell == Cell.SWAMP):

            if next_state == state:
                return 0.25

            if action == Action.NORTH:
                if next_state[0] == state[0]-1 and next_state[1] == state[1]:
                    return 0.5
            
            if action == Action.SOUTH:
                if next_state[0] == state[0]+1 and next_state[1] == state[1]:
                    return 0.5

            if action == Action.WEST:
                if next_state[0] == state[0] and next_state[1] == state[1]-1:
                    return 0.5

            if action == Action.EAST:
                if next_state[0] == state[0] and next_state[1] == state[1]+1:
                    return 0.5
            
            if self.manh_distance(state, next_state) == 1:
                return 1/12

            return 0



    def stage_reward(self, state: State, action: Action, next_state: State) -> float:

        curr_cell = self.grid[state]

        if curr_cell == Cell.GOAL:
                return +10

        if curr_cell == Cell.SWAMP:
            return -2

        return -1


class GridMdpSolver(ABC):
    @staticmethod
    @abstractmethod
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        pass
