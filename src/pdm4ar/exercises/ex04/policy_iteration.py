from typing import Tuple

import numpy as np

from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import Action, Cell, ValueFunc, Policy
from pdm4ar.exercises_def.ex04.utils import time_function


class PolicyIteration(GridMdpSolver):
    @staticmethod
    @time_function
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        
        # setting parameters
        N = grid_mdp.grid.shape[0]
        M = grid_mdp.grid.shape[1]
        gamma = grid_mdp.gamma

        # Bulding state set S and save start
        S = []
        start = (0,0)
        for i in range(N):
            for j in range(M):
                S.append((i,j))
                if grid_mdp.grid[(i,j)] == Cell.START:
                    start = (i,j)

        # ---------------------------------------------
        # 0. POLICY DEFINITION - build policy
        # Go North. If you are on the top edge, go South.

        policy = np.zeros((N,M))
        for j in range(M):
            policy[0,j] = Action.SOUTH

        V = np.zeros_like(grid_mdp.grid).astype(float)

        changing = True
        it = 0
        while changing:

            # ---------------------------------------------
            # 1. POLICY EVALUATION - build V
            # ---------------------------------------------

            # Initialize V = 0
            epsilon = 0.0000001
            max_deltaV = 1.0

            while max_deltaV > 0.2**it:

                max_deltaV = 0.0

                for state in S:

                    # selecting action according to the policy
                    action = policy[state]

                    # calculating expected value from state s with policy
                    exp_value = 0.0
                    for next_state in grid_mdp.get_reachable(state, action, start):
                        prob_to_go = grid_mdp.get_transition_prob(state, action, next_state)
                        if grid_mdp.out_of_bounds(next_state):
                            next_state = start   # to calculate stage reward correctly
                        stage_reward = grid_mdp.stage_reward(state, action, next_state)
                        exp_value += prob_to_go*(stage_reward + gamma*V[next_state])
                    
                    currDeltaV = np.abs(V[state]-exp_value)
                    if currDeltaV > max_deltaV:
                        max_deltaV = currDeltaV
                    V[state] = exp_value



            # ---------------------------------------------
            # ROLL OUT - Improve policy
            # ---------------------------------------------

            best_action = Action.SOUTH # random action
            changing = False

            # build new policy
            for state in S:
                maximum = -np.Inf
                for action in grid_mdp.get_actions(state):
                
                    # calculating expected value corrisponding to the action
                    exp_value = 0.0
                    for next_state in grid_mdp.get_reachable(state, action, start):
                        prob_to_go = grid_mdp.get_transition_prob(state, action, next_state)
                        if grid_mdp.out_of_bounds(next_state):
                            next_state = start   # to calculate stage reward correctly
                        stage_reward = grid_mdp.stage_reward(state, action, next_state)
                        exp_value += prob_to_go*(stage_reward + gamma*V[next_state])

                    # saving action in the policy if it's the best
                    if exp_value > maximum:
                        maximum = exp_value
                        best_action = action
                
                if policy[state] != best_action:
                    changing = True
                policy[state] = best_action

            it = it+2

        return V, policy
