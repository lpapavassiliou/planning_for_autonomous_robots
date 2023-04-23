import numpy as np

from pdm4ar.exercises_def.ex07.structures import (
    ProblemVoyage,
    OptimizationCost,
    Island,
    Constraints,
    Feasibility,
    SolutionVoyage,
)

##
from scipy.optimize import milp, LinearConstraint, Bounds

def get_islands_idx(id_arch, K):
    return range((id_arch-1)*K+1, id_arch*K+1)

def indx_vec(position, lenght):
    # returns a vector of dimension of x (I-2 for now) which has a 1 in position i
    vec = np.zeros([lenght])
    vec[position] = 1
    return vec


def solve_optimization(problem: ProblemVoyage) -> SolutionVoyage:
    """
    Solve the optimization problem enforcing the requested constraints.

    Parameters
    ---
    problem : ProblemVoyage
        Contains the problem data: cost to optimize, starting crew, tuple of islands,
        and information about the requested constraint (the constraints not set to `None` +
        the `voyage_order` constraint)

    Returns
    ---
    out : SolutionVoyage
        Contains the feasibility status of the problem, and the optimal voyage plan
        as a list of ints if problem is feasible, else `None`.
    """

    # PROBLEM DEFINITION

    # I = 8
    # arch = [0, 1, 1, 2, 2, 3, 3, 4]
    # n = [0, 2, 3, 3, 3, 2, 3, 0]
    # h = [0, 10, -12, -4, -1, 8, -9, 0]
    # delta = [9, 9, 9, 9, 9, 9, 7, 9]
    # alpha = [16, 15, 15, 10, 15, 10, 10, 10]
    # x = [0, 1, 2, 3, 4, 5, 6, 7]
    # y = [0, 1, 2, 3, 4, 5, 6, 7]
    # islands = []
    # for i in range(I):
    #     islands.append(Island(i,arch[i],x[i],y[i],delta[i],alpha[i], n[i], h[i]))

    # start_crew = 10
    # optimization_cost = OptimizationCost.min_max_sailing_time
    # constr = Constraints(None,None,None,None,None)
    # problem = ProblemVoyage(optimization_cost, start_crew, islands, constr)


    # GET PARAMETERS

    I = len(problem.islands)
    N = problem.islands[-1].arch + 1
    K = int((I-2)/(N-2))

    # print('I is ' + str(I))
    # print('N is ' + str(N))
    # print('K is ' + str(K))

    n = np.array([problem.islands[i].nights for i in range(1,I-1)])  # we dont care about first and last island
    h = np.array([problem.islands[i].delta_crew for i in range(1, I-1)])
    delta = np.array([problem.islands[i].departure for i in range(1, I-1)])
    alpha = np.array([problem.islands[i].arrival for i in range(1, I-1)])
    coord_x = np.array([problem.islands[i].x for i in range(I)])
    coord_y = np.array([problem.islands[i].y for i in range(I)])
    


    D = {}
    for arch in range(1,N-2):
        for i in get_islands_idx(arch,K):
            for j in get_islands_idx(arch+1,K):
                D[(i,j)] = np.abs(problem.islands[i].x-problem.islands[j].x)+np.abs(problem.islands[i].y-problem.islands[j].y)
    for j in get_islands_idx(1,K):
        D[(0,j)] = np.abs(problem.islands[0].x-problem.islands[j].x)+np.abs(problem.islands[0].y-problem.islands[j].y)
    for i in get_islands_idx(N-2,K):
        D[(i,I-1)] = np.abs(problem.islands[i].x-problem.islands[I-1].x)+np.abs(problem.islands[i].y-problem.islands[I-1].y)
    
    M = 1000   # large constant



    # DECIDE NUMBER OF HELPING VARIABLES
    num_help_var = 0
    if problem.optimization_cost == OptimizationCost.min_total_travelled_L1_distance:
        num_help_var += 2*(N-1)   # x_arch, y_arch
    if problem.optimization_cost == OptimizationCost.min_max_sailing_time:
        num_help_var += N-1   # t_arch
        num_help_var += 1     # t_max


#########################################################################################
    # IMPLEMENT OBJECTIVE

    if problem.optimization_cost == OptimizationCost.min_total_nights:
        c = n
        c = np.hstack((c, np.zeros([num_help_var])))

    if problem.optimization_cost == OptimizationCost.max_final_crew:
        c = -h
        c = np.hstack((c, np.zeros([num_help_var])))

    if problem.optimization_cost == OptimizationCost.min_total_sailing_time:
        # since delta0 and alphaN-1 are constant, they can be removed.
        c = alpha-delta
        c = np.hstack((c, np.zeros([num_help_var])))
    
    if problem.optimization_cost == OptimizationCost.min_total_travelled_L1_distance:
        c = np.hstack((np.zeros(I-2),np.ones(2*(N-1))))
        

    if problem.optimization_cost == OptimizationCost.min_max_sailing_time:
        c = np.zeros(I-2 + num_help_var)
        c[-1] = 1
        

###################################################################################
    # IMPLEMENT CONSTRAINTS
    constraint_list = []

    # travel order constraint
    A = np.zeros([N-2, (N-2)*K])
    for i in range(N-2):
        for j in range(K):
            A[i, j+K*i] = 1
    A = np.hstack((A, np.zeros([A.shape[0], num_help_var])))
    b_u = np.ones([N-2])
    b_l = b_u
    constraint_list.append(LinearConstraint(A, b_l, b_u))
    
    
    if problem.constraints.min_nights_individual_island is not None:
        n_min = problem.constraints.min_nights_individual_island
        A = n_min*np.eye(I-2, I-2)
        A = np.hstack((A, np.zeros([A.shape[0], num_help_var])))
        b_u = n
        b_l = np.full_like(b_u, -np.inf)
        constraint_list.append(LinearConstraint(A, b_l, b_u))

    if problem.constraints.min_total_crew is not None:
        min_crew = problem.constraints.min_total_crew
        init_crew = problem.start_crew
        h_0 = problem.islands[0].delta_crew
        A = np.zeros([I-2, I-2])
        for i in range(I-2):
            for j in range(I-2):
                if j<=i:
                    A[i,j] = h[j]
        A = np.hstack((A, np.zeros([A.shape[0], num_help_var])))
        b_l = (min_crew-init_crew-h_0)*np.ones([I-2])
        b_u = np.full_like(b_l, np.inf)
        constraint_list.append(LinearConstraint(A,b_l, b_u))
            

    if problem.constraints.max_total_crew is not None:
        max_crew = problem.constraints.max_total_crew
        init_crew = problem.start_crew
        h_0 = problem.islands[0].delta_crew
        A = np.zeros([I-2, I-2])
        for i in range(I-2):
            for j in range(I-2):
                if j<=i:
                    A[i,j] = h[j]
        A = np.hstack((A, np.zeros([A.shape[0], num_help_var])))
        b_u = (max_crew-init_crew-h_0)*np.ones([I-2])
        b_l = np.full_like(b_u, -np.inf)
        constraint_list.append(LinearConstraint(A,b_l, b_u))


    if problem.constraints.max_duration_individual_journey is not None:
        max_t = problem.constraints.max_duration_individual_journey
        for id_arch in range(1, N-2):
            for j in get_islands_idx(id_arch+1, K):
                j = j-1
                for i in get_islands_idx(id_arch,K):
                    i = i-1
                    A = (alpha[j]-delta[i])*indx_vec(i,I-2) + M*indx_vec(j,I-2)
                    A = np.hstack((A, np.zeros([num_help_var])))
                    b_u = max_t + M
                    b_l = -np.inf
                    constraint_list.append(LinearConstraint(A,b_l, b_u))
    
        # first travel
        for j in get_islands_idx(1,K):
            j = j-1
            delta_0 = problem.islands[0].departure
            A = (alpha[j]-delta_0)*indx_vec(j,I-2)
            A = np.hstack((A, np.zeros([num_help_var])))
            b_u = max_t
            b_l = -np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))

        # last travel
        for i in get_islands_idx(N-2,K):
            i = i-1
            alpha_last = problem.islands[-1].arrival
            A = (alpha_last-delta[i])*indx_vec(i,I-2)
            A = np.hstack((A, np.zeros([num_help_var])))
            b_u = max_t
            b_l = -np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))


    if problem.constraints.max_L1_distance_individual_journey is not None:

        max_d = problem.constraints.max_L1_distance_individual_journey

        for id_arch in range(1, N-2):
            for j in get_islands_idx(id_arch+1, K):
                for i in get_islands_idx(id_arch,K):
                    A = M*(indx_vec(i-1,I-2)+indx_vec(j-1,I-2))
                    A = np.hstack((A, np.zeros([num_help_var])))
                    b_u = max_d - D[(i,j)]+ 2*M
                    b_l = -np.inf
                    constraint_list.append(LinearConstraint(A,b_l, b_u))

        # first travel
        for j in get_islands_idx(1,K):
            A = M*indx_vec(j-1,I-2)
            A = np.hstack((A, np.zeros([num_help_var])))
            b_u = max_d - D[(0,j)] + M
            b_l = -np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))

        # last travel
        for i in get_islands_idx(N-2,K):
            A = M*indx_vec(i-1,I-2)
            A = np.hstack((A, np.zeros(num_help_var)))
            b_u = max_d - D[(i,I-1)] + M
            b_l = -np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))
        
###############################################################################################
    # IMPLEMENT CONSTRAINTS FOR HELPING VARS

    if problem.optimization_cost == OptimizationCost.min_total_travelled_L1_distance:
        # constraints for x_arch and y_arch

        for id_arch in range(1, N-2):
            
            vec_a_i = np.zeros(I-2+num_help_var)
            vec_d_i = np.zeros(I-2+num_help_var)
            for i in get_islands_idx(id_arch, K):
                vec_a_i[i-1] = coord_x[i]
                vec_d_i[i-1] = coord_y[i]
            vec_x_j = np.zeros(I-2+num_help_var)
            vec_y_j = np.zeros(I-2+num_help_var)
            for j in get_islands_idx(id_arch+1, K):
                vec_x_j[j-1] = coord_x[j]
                vec_y_j[j-1] = coord_y[j]

            # x_arch
            vec_arch_x = indx_vec(I-2+id_arch, I-2+num_help_var)
            A = vec_arch_x + vec_x_j - vec_a_i
            b_l = 0
            b_u = np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))
            A = vec_arch_x - vec_x_j + vec_a_i
            b_l = 0
            b_u = np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))

            # y_arch
            vec_arch_y = indx_vec(I-2+(N-1)+id_arch, I-2+num_help_var)
            A = vec_arch_y + vec_y_j - vec_d_i
            b_l = 0
            b_u = np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))
            A = vec_arch_y - vec_y_j + vec_d_i
            b_l = 0
            b_u = np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))


        id_arch = 0
        vec_x_j = np.zeros(I-2+num_help_var)
        vec_y_j = np.zeros(I-2+num_help_var)
        for j in get_islands_idx(id_arch+1, K):
            vec_x_j[j-1] = coord_x[j]
            vec_y_j[j-1] = coord_y[j]

        
        # x_arch
        vec_arch_x = indx_vec(I-2+id_arch, I-2+num_help_var)
        A = vec_arch_x + vec_x_j
        b_l = coord_x[0]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))
        A = vec_arch_x - vec_x_j
        b_l = - coord_x[0]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))

        # y_arch
        vec_arch_y = indx_vec(I-2+(N-1)+id_arch, I-2+num_help_var)
        A = vec_arch_y+vec_y_j
        b_l = coord_y[0]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))
        A = vec_arch_y-vec_y_j
        b_l = -coord_y[0]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))
        

        
        id_arch = N-2
        vec_a_i = np.zeros(I-2+num_help_var)
        vec_d_i = np.zeros(I-2+num_help_var)
        for i in get_islands_idx(N-2,K):
            vec_a_i[i-1] = coord_x[i]
            vec_d_i[i-1] = coord_y[i]

        # x_arch
        vec_arch_x = indx_vec(I-2+id_arch, I-2+num_help_var)
        A = vec_arch_x + vec_a_i
        b_l = coord_x[I-1]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))
        A = vec_arch_x - vec_a_i
        b_l = - coord_x[I-1]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))

        # y_arch
        vec_arch_y = indx_vec(I-2+(N-1)+id_arch, I-2+num_help_var)
        A = vec_arch_y+vec_d_i
        b_l = coord_y[I-1]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))
        A = vec_arch_y-vec_d_i
        b_l = -coord_y[I-1]
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))



    
    if problem.optimization_cost == OptimizationCost.min_max_sailing_time:

        # ----------------------------------------------------------------
        # 1. costraining t_arch

        for id_arch in range(1, N-2):
            
            vec_d_i = np.zeros(I-2+num_help_var)
            vec_a_j = np.zeros(I-2+num_help_var)
            for i in get_islands_idx(id_arch, K):
                vec_d_i[i-1] = delta[i-1]
            for j in get_islands_idx(id_arch+1, K):
                vec_a_j[j-1] = alpha[j-1]

            vec_arch = indx_vec(I-2+id_arch, I-2+num_help_var)
            A = vec_arch - vec_a_j + vec_d_i
            b_l = 0
            b_u = np.inf
            constraint_list.append(LinearConstraint(A,b_l, b_u))

        id_arch = 0
        d_0 = problem.islands[0].departure
        vec_a_j = np.zeros(I-2+num_help_var)
        vec_arch = indx_vec(I-2+id_arch, I-2+num_help_var)
        for j in get_islands_idx(1,K):
            vec_a_j[j-1] = alpha[j-1]
        A = vec_arch - vec_a_j
        b_l = -d_0
        b_u = np.inf
        # print('b_l, A, b_u')
        # print(b_l)
        # print(A)
        # print(b_u)
        constraint_list.append(LinearConstraint(A,b_l, b_u))

        id_arch = N-2
        a_last = problem.islands[I-1].arrival
        vec_d_i = np.zeros(I-2+num_help_var)
        vec_arch = indx_vec(I-2+id_arch, I-2+num_help_var)
        for i in get_islands_idx(N-2,K):
            vec_d_i[i-1] = delta[i-1]
        A = vec_arch + vec_d_i
        b_l = a_last
        b_u = np.inf
        constraint_list.append(LinearConstraint(A,b_l, b_u))

        # ---------------------------------------------------
        # 2. constraining variable t_max
        A_x = np.zeros([N-1,I-2])
        A_t = np.eye(N-1,N-1)
        A_tmax = np.full([N-1,1],-1.0)
        A = np.hstack((A_x,A_t,A_tmax))
        b_l = -np.inf
        b_u = 0
        constraint_list.append(LinearConstraint(A,b_l, b_u))




        
######################################################################
    # INTEGRALITY AND BOUNDS
    # x -> {0,1}
    # hv -> [0, +Inf]

    integrality_x = np.ones(I-2)
    integrality_hv = np.zeros(num_help_var)
    integrality = np.hstack((integrality_x, integrality_hv))

    lb_x = np.zeros_like(integrality_x)
    ub_x = np.ones_like(integrality_x)
    lb_hv = np.zeros_like(integrality_hv)
    ub_hv = np.full_like(integrality_hv, np.inf)
    lb = np.hstack((lb_x,lb_hv))
    ub = np.hstack((ub_x,ub_hv))
    bounds = Bounds(lb=lb,ub=ub)
    result = milp(c=c, constraints=constraint_list, integrality=integrality, bounds=bounds)

    if result.success:
        # print('\nresult found!')
        feasibility = Feasibility.feasible
        voyage_plan = [0]
        for i in range(I-2):
            if result.x[i] > 0.1:
                voyage_plan.append(problem.islands[i+1].id)
        voyage_plan.append(I-1)


        # print('voyage plan')
        # print(voyage_plan)
        # print('x')
        # print(result.x)


    else:
        feasibility = Feasibility.unfeasible
        # print('result not found')
        # print(result.status)
        voyage_plan = None

    return SolutionVoyage(feasibility, voyage_plan)
