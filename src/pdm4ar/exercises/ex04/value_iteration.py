from typing import Tuple

import numpy as np
from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import Action, Cell, Policy, ValueFunc
from pdm4ar.exercises_def.ex04.utils import time_function


class ValueIteration(GridMdpSolver):
    @staticmethod
    @time_function
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        # ----------------------------------------------------
        # initialization: V(s) = 0 for all s in S
        # indirectly builds set S which are the keys of the dictionary V
        # finding start state for out of bound cases

        N = grid_mdp.grid.shape[0]
        M = grid_mdp.grid.shape[1]
        S = []
        start = (0,0)
        for i in range(N):
            for j in range(M):
                S.append((i,j))
                if grid_mdp.grid[(i,j)] == Cell.START:
                    start = (i,j)

        # ----------------------------------------------------
        # Recursion: build Vnew and update V
        V = np.zeros_like(grid_mdp.grid).astype(float)
        policy = np.zeros_like(grid_mdp.grid).astype(int)
        gamma = grid_mdp.gamma
        epsilon = 0.0000001
        max_deltaV = 1.0

        it = 0
        while max_deltaV > epsilon:

            max_deltaV = 0.0

            for state in S:
                maximum = -np.Inf
                for action in grid_mdp.get_actions(state):
                    exp_value = 0.0
                    for next_state in grid_mdp.get_reachable(state, action, start):
                        prob_to_go = grid_mdp.get_transition_prob(state, action, next_state)
                        if grid_mdp.out_of_bounds(next_state):
                            next_state = start   # to calculate stage reward correctly
                        stage_reward = grid_mdp.stage_reward(state, action, next_state)
                        exp_value += prob_to_go*(stage_reward + gamma*V[next_state])

                    if exp_value > maximum:
                        maximum = exp_value
                        policy[state] = action

                old_value = V[state]
                V[state] = maximum
                currDeltaV = np.abs(V[state]-old_value)
                
                if currDeltaV > max_deltaV:
                    max_deltaV = currDeltaV
            it += 1
            
        return V, policy