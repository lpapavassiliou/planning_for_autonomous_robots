import random
from dataclasses import dataclass
from typing import Sequence

from commonroad.scenario.lanelet import LaneletNetwork
from dg_commons import PlayerName
from dg_commons.sim.goals import PlanningGoal
from dg_commons.sim import SimObservations, InitSimObservations
from dg_commons.sim.agents import Agent
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.vehicle import VehicleCommands
from dg_commons.sim.models.vehicle_structures import VehicleGeometry
from dg_commons.sim.models.vehicle_utils import VehicleParameters

##
import numpy as np
from shapely.geometry import Polygon, LineString, LinearRing, Point
import matplotlib.pyplot as plt
from shapely.strtree import STRtree


@dataclass(frozen=True)
class Pdm4arAgentParams:
    bounds_map :float
    num_goal_samples: float
    def __init__(self, bounds_map, num_goal_samples, v_nom):
        self.bounds_map = bounds_map
        self.num_goal_samples = num_goal_samples
        self.v_nom = v_nom
        


class Graph:

    def __init__(self) -> None:
        self.E = {}
        self.W = {}

    def add_point(self, point, n):
        neighbours_list = hf.neighbours(self.E.keys(), point, n)
        self.E[point] = []
        for neighbour in neighbours_list:
            self.add_edge(point, neighbour)
        
    def add_point_no_collision(self, point, n, radius, obstacles):
        neighbours_list = hf.neighbours(self.E.keys(), point, n)
        self.E[point] = []
        for neighbour in neighbours_list:
            if not hf.path_collision_check_safety_certificate([point, neighbour], radius, obstacles):
                self.add_edge(point, neighbour)

    def a_star(self, start, goal):
        Q = {start: 0}
        costToReach = {}
        for node in self.E.keys():
            costToReach[node] = np.inf
        costToReach[start]=0
        Parent = {start:None}
        path = []
        goalReached = False
        while len(Q) != 0 and goalReached is False:
            chosenNode = min(Q, key=Q.get)
            _ = Q.pop(chosenNode)
            if chosenNode == goal:
                goalReached = True
                i = chosenNode
                while Parent[i] is not None:
                    path.insert(0, i)
                    i = Parent[i]
                path.insert(0, start)
            else:
                for reachable in self.E[chosenNode]:
                    tempCostToReach = costToReach[chosenNode] + self.W[(chosenNode,reachable)]
                    if tempCostToReach < costToReach[reachable]:
                        costToReach[reachable] = tempCostToReach
                        Parent[reachable] = chosenNode
                        Q[reachable] = tempCostToReach + hf.heuristic(reachable, goal)
        return path
    
    def a_star_with_max_angle(self, start_point, psi, goal_point):
        while(True):
            path = self.a_star(start_point, goal_point)
            valid = True
            vectors = [hf.vector(1, psi)]
            for idx, _ in enumerate(path[:-1]):
                vectors.append(np.array([path[idx+1][0]-path[idx][0], path[idx+1][1]-path[idx][1]]))
            for indx, _ in enumerate(vectors[:-1]):
                if hf.normalized_cross(vectors[indx], vectors[indx+1])<0.55:  #0.55
                    self.remove_edge(path[indx], path[indx+1])
                    #hf.plot_path((path[indx], path[indx+1]))
                    valid = False
                    break
            if valid:
                break
        return path

    def add_edge(self, point1, point2):
        if point1 in self.E.keys():
            self.E[point1].append(point2)
        else:
            self.E[point1] = [point2]
        if point2 in self.E.keys():
            self.E[point2].append(point1)
        else:
            self.E[point2] = [point1]
        self.W[(point1, point2)] = hf.distance(point1, point2)
        self.W[(point2, point1)] = self.W[point1, point2]

    def remove_edge(self, node1, node2):
        # removing edge 1->2
        if node1 in self.E.keys():
            if node2 in self.E[node1]:
                self.E[node1].remove(node2)
        # removing edge 2->1
        if node2 in self.E.keys():
            if node1 in self.E[node2]:
                self.E[node2].remove(node1)

    def get_PRM(self, start, goal_point, polygon_map, radius, stat_obs):
        start_point = start[0]
        psi = start[1]

        # FIRST PATH
        # ---------------------------------------------------------
        while True:
            path = self.a_star(start_point, goal_point)
            collision_segments =  hf.path_collision_check_safety_certificate(path, radius, stat_obs['lines'])
            if len(collision_segments)==0:
                break
            for i in collision_segments:
                self.remove_edge(path[i], path[i+1])
        
        if len(path)==0:
            return []
        #print('*** first path found. ***')
        #print('old path had lenght ' + str(len(path)))
        path = hf.enrich(path)
        #print('new path has lenght ' + str(len(path)))

        # SECOND PATH
        # ----------------------------------------------------
        total_static_obs = stat_obs['polygons']+stat_obs['rings']+stat_obs['lines']
        new_graph = hf.graph_around_path(path, polygon_map, radius, total_static_obs)
        new_graph.add_point_no_collision(goal_point, 2, radius, total_static_obs)
        new_graph.add_point_no_collision(start_point, 2, radius, total_static_obs)
        #print('new grap built')

        for i in range(3):
            new_path = new_graph.a_star_with_max_angle(start_point, psi, goal_point)
            if len(new_path)!= 0:
                break
            #print('adding more points around path...')
            radius -= 0.07
            new_graph.add_points_around_path(path, polygon_map, radius, total_static_obs)

        print('second path found with lenght ' + str(len(new_path)))

        return new_path

    def add_200_points(self, polygon_map: Polygon):
        squared_map = hf.build_box(polygon_map)
        for n in range(2, 200):
            while True:
                new_point = hf.shoot(squared_map)
                if polygon_map.intersects(Point(new_point)):
                    break
            self.add_point(new_point, n)
    

    def PRM_complete_old(self, start, goal_point, polygon_map, radius, stat_obs):

        start_point = start[0]
        psi = start[1]

        # FIRST PATH
        # ---------------------------------------------------------
        f_self = self
        it = 0
        while True:
            while True:
                path = f_self.a_star(start_point, goal_point)
                collision_segments =  hf.path_collision_check_safety_certificate(path, radius, stat_obs['lines'])
                if len(collision_segments)==0:
                    break
                for i in collision_segments:
                    f_self.remove_edge(path[i], path[i+1])
            if len(path) != 0:
                break
            f_self.add_200_points(polygon_map)
            print('adding more points for graph 1')
            it += 1

        print(len(path))
        path = hf.enrich(path, 1)
 
        # SECOND PATH
        # ----------------------------------------------------
        total_static_obs = stat_obs['polygons']+stat_obs['rings']+stat_obs['lines']
        d = 5
        exp = 0.2
        N = 10

        while True:
            new_graph = hf.graph_around_path_hlt(path, polygon_map, radius, total_static_obs, d, exp, N)
            new_graph.add_point_no_collision(goal_point, 10, radius, total_static_obs)
            new_graph.add_point_no_collision(start_point, 10, radius, total_static_obs)

            new_path = new_graph.a_star_with_max_angle(start_point, psi, goal_point)
            if len(new_path)!= 0:
                break
            else:
                #print('------------')
                #print('increasing range')
                d = min(d+6, 35)
                N = N*1.5
                #print('d is ' + str(d))
                #hf.plot_graph(new_graph)
                #hf.save()

        #print('second path found with lenght ' + str(len(new_path)))
        #hf.plot_path(new_path, 'black', 0.5)
        new_path = hf.add_more_points_to_path(new_path, 0.75)
        new_path = hf.smoothing(new_path)
        #hf.plot_path(new_path, 'orange', 1)
        #hf.save()

        return new_path

    def add_points_around_path(self, path, polygon_map: Polygon, radius, obstacles):
        for point in path:
            square = hf.square_around(point, 12)
            for n in range(2, 4):
                while True:
                    new_point = hf.shoot(square)
                    valid = False
                    if polygon_map.intersects(Point(new_point)):
                        valid = True
                        for obs in obstacles:
                            if obs.intersects(Point(new_point).buffer(radius)):
                                valid = False
                                break
                    if valid:
                        break
                self.add_point_no_collision(new_point, n, radius, obstacles)

class Pdm4arAgent(Agent):
    """This is the PDM4AR agent.
    Do *NOT* modify the naming of the existing methods and the input/output types.
    Feel free to add additional methods, objects and functions that help you to solve the task"""

 
    def __init__(self,
                 sg: VehicleGeometry,
                 sp: VehicleParameters
                 ):
        self.sg = sg
        self.sp = sp
        self.name: PlayerName = None
        self.goal: PlanningGoal = None
        self.lanelet_network: LaneletNetwork = None
        self.static_obstacles: Sequence[StaticObstacle] = None
        # feel free to remove/modify  the following
        #self.params = Pdm4arAgentParams()
        self.stat_obs = None  #già c'era!
        self.nominal_radius = None
        self.polygon_map = None
        self.trajectory = None
        self.graph = None
        self.goal_point = None
        self.obstacles_tree = None

    def on_episode_init(self, init_obs: InitSimObservations):
        """This method is called by the simulator at the beginning of each episode."""
        self.name = init_obs.my_name
        self.goal = init_obs.goal
        #self.lanelet_network = init_obs.dg_scenario.lanelet_network
        self.static_obstacles = list(init_obs.dg_scenario.static_obstacles.values())

        # Shapely: 7 poligoni, 4 line arring (linee chiuse), 7 line string (linee aperte, ordinate in senso orario partendo dal buco a sx della macchinina)
        # Puoi creare un poligono unendo le linee aperte: prendi gli estremi dei segmentini di ogni linea e collegali. Con within controlla se il punto è dentro
        # Il poligono fornisce min x, min y, max x, max y
        # self.sg # parametri macchina
        # self.sp #parametri attuatori

        # 0. VECHICLE
        self.nominal_radius = 1.4*self.sg.width
        radius = self.nominal_radius

        # CREATING STATIC OBSTACLES DICTIONARY
        # ------------------------------------------------------
        stat_obs = {
            'polygons' : [],
            'lines' : [],
            'rings' : []
        }
        for obs in self.static_obstacles:
            if isinstance(obs.shape, Polygon):
                stat_obs['polygons'].append(obs.shape)
            elif isinstance(obs.shape, LinearRing):
                stat_obs['rings'].append(obs.shape)
            elif isinstance(obs.shape, LineString):
                stat_obs['lines'].append(obs.shape)
        self.stat_obs = stat_obs
        self.obstacle_tree = STRtree(stat_obs['lines']+stat_obs['rings']+stat_obs['polygons'])

        # CREATING THE POLYGON MAP
        # ------------------------------------------------------
        verteces = []
        for line in stat_obs['lines']:
            assert(isinstance(line, LineString))
            for point in line.coords:
                verteces.append(point)
        polygon_map = Polygon(verteces)
        self.polygon_map = polygon_map
        #hf.plot_map_and_obs(polygon_map, stat_obs, self.goal.goal)


        # SAMPLING GOAL POINT
        # ------------------------------------------------------
        self.goal_point = hf.sample_goal(self.goal.goal, polygon_map,radius, stat_obs['polygons']+stat_obs['rings']+stat_obs['lines'])
        #hf.save()
        #self.goal_point = (40, 0)
        #self.goal_point = (-33, -32)

        # # GRAPH GENERATOR (put in get_commands)
        # # -------------------------------------------------------
        # start_point = (-10, -25)
        # psi = np.pi/3
        # start = (start_point, psi)
        # hf.plot_pose(start_point, psi, 'purple')
        # hf.plot_point(self.goal_point)
        # print('generating graph...')

        # # TRAJECTORY GENERATOR (put in get_commands)
        # # -------------------------------------------------------
        # while True:
        #     print('NEW ATTEMPT')
        #     graph = hf.generate_graph(start_point, polygon_map)
        #     graph.add_point_no_collision(self.goal_point, 2, radius, stat_obs['lines'])
        #     self.graph = graph
        #     for i in range(3):
        #         path = self.graph.get_PRM(start, self.goal_point, polygon_map, radius, stat_obs)
        #         if len(path) != 0:
        #             break
        #         if i == 1:
        #             radius = self.nominal_radius*0.85
        #         self.graph.add_200_points(polygon_map)
        #         print('    adding more points and reducing r...')
        #     if len(path) != 0:
        #         break
        # radius = self.nominal_radius


        # # UPDATING TRAJECTORY (put in get_commands)
        # # --------------------------------------------------------
        # hf.plot_path(path)
        # path = hf.add_more_points_to_path(path, 3)
        # path = hf.smoothing(path)
        # hf.plot_path(path, 'red')
        
        # c_pos = (0, -10)
        # c_psi = np.pi/2
        # hf.plot_pose(c_pos, c_psi)
        # trajectory_graph = Graph()
        # for idx, point in enumerate(path[:-1]):
        #     trajectory_graph.add_edge(point, path[idx+1])
        # trajectory_graph.add_point_no_collision(c_pos, c_psi, radius, stat_obs['polygons']+stat_obs['rings']+stat_obs['lines'])
        # hf.plot_graph(trajectory_graph)
        # self.trajectory = trajectory_graph.a_star_with_max_angle(c_pos, c_psi, self.goal_point)
        # hf.plot_path(self.trajectory)
        # self.trajectory = hf.add_more_points_to_path(self.trajectory, 1)
        # self.trajectory = hf.smoothing(self.trajectory)
        # hf.plot_path(self.trajectory, 'green', 1)
        # hf.save()
        # exit()



    def get_commands(self, sim_obs: SimObservations) -> VehicleCommands:
        """ This method is called by the simulator at each time step.
        For instance, this is how you can get your current state from the observations:
        my_current_state: VehicleState = sim_obs.players[self.name].state

        :param sim_obs:
        :return:
        """

        # READ PARAMS
        # ------------------------------------------------------
        radius = self.nominal_radius
        cog = (sim_obs.players[self.name].state.x, sim_obs.players[self.name].state.y)
        v = sim_obs.players[self.name].state.vx
        delta = sim_obs.players[self.name].state.delta
        psi = sim_obs.players[self.name].state.psi
        p_np = np.array(cog) - hf.vector(self.sg.lr, psi)
        p = (p_np[0], p_np[1])
        wb = self.sg.wheelbase

        #hf.plot_polygon(polygon_car)
        ##hf.plot_point_red(p, 'green', 2)

        # dyn_obs = []
        # for player in sim_obs.players.keys():
        #     if player != self.name:
        #         cog2 = sim_obs.players[player].occupancy.centroid
        #         if v < 0.1 and cog2.distance(Point(cog)) < 6:
        #             dyn_obs.append(sim_obs.players[player].occupancy)
        #             # print('considering player in path')


        # UPDATING TRAJECTORY
        # --------------------------------------------------------
        if self.trajectory is not None and len(self.trajectory) != 0:
            first_pursuit_point = None
            for point in reversed(hf.add_more_points_to_path(self.trajectory, 0.5)):
                if hf.distance(point, p) > 4 and hf.distance(point, p) < 5:
                    first_pursuit_point = point
                    break
            distance_from_trajectory = Point(p).distance(LineString(self.trajectory))

            if first_pursuit_point is None:
                return VehicleCommands(-0.5*v,0)

            if distance_from_trajectory > 1:
                #print('too far! recalibrating path')
                trajectory_graph = Graph()
                for idx, point in enumerate(self.trajectory[:-1]):
                    trajectory_graph.add_edge(point, self.trajectory[idx+1])
                trajectory_graph.add_point_no_collision(p, 40, radius, self.stat_obs['polygons']+self.stat_obs['rings']+self.stat_obs['lines'])
                ## add dyn_obs?
                path = trajectory_graph.a_star_with_max_angle(p, psi, self.goal_point)
                path = hf.add_more_points_to_path(path, 0.75)
                path = hf.smoothing(path)
                self.trajectory = path

        # TRAJECTORY GENERATOR
        # -------------------------------------------------------
        if self.trajectory is None or len(self.trajectory) == 0:

            print(self.name + 'computing trajectory')
            self.trajectory = hf.PRM_complete([p, psi], self.goal_point, self.polygon_map, radius, self.stat_obs)

        #     start_point = p
        #     start = (start_point, psi)
        #     while True:
        #         print('NEW ATTEMPT')
        #         graph = hf.generate_graph(start_point, self.polygon_map)
        #         graph.add_point_no_collision(self.goal_point, 2, radius, self.stat_obs['lines'])
        #         for i in range(3):
        #             path = graph.get_PRM(start, self.goal_point, self.polygon_map, radius, self.stat_obs)
        #             if len(path) != 0:
        #                 break
        #             graph.add_200_points(self.polygon_map)
        #             #print('    adding more points to graph1 and reducing r...')
        #         if len(path) != 0:
        #             break
        #     radius = self.nominal_radius
        #     path = hf.add_more_points_to_path(path, 3)
        #     path = hf.smoothing(path)
        #     self.trajectory = path
        #     hf.plot_path(self.trajectory, 'orange', 0.5)
        #     hf.save()
        # hf.plot_point_red(p)

        # PURE PURSUIT CONTROLLER
        # -------------------------------------------------------
        l_d = 5
        pursuit_point = None
        idx_pp = None
        for idx, point in enumerate(reversed(hf.add_more_points_to_path(self.trajectory, 0.5))):
            if hf.distance(point, p) > l_d-0.5 and hf.distance(point, p) < l_d + 0.5:
                pursuit_point = point
                idx_pp = idx
                break
        if pursuit_point is None:
            return VehicleCommands(-0.5*v,0)
        ##hf.plot_point_red(pursuit_point)

        # delta reference
        beta = hf.angle(p, pursuit_point)
        g = np.array(pursuit_point)
        pg_W = g-p_np
        L = np.linalg.norm(pg_W)
        R_RW = hf.rot_mat_RW(psi)
        pg_R = R_RW @ pg_W
        r = (L**2)/(2*np.abs(pg_R[1]))
        delta_ref = np.arctan2(wb,r)
        delta_ref *= np.sign(beta-psi)

        # velocity reference
        v_ref = 6

        if self.trajectory is not None and len(self.trajectory) != 0:
            distance_from_trajectory = Point(p).distance(LineString(self.trajectory))
            if distance_from_trajectory > 0.5:
                v_ref = 0.66*v_ref
        angle_error = hf.angle_error(hf.angle(p, pursuit_point), psi)

        # idx = idx_pp
        # prec_angle = psi
        # sum_angle = 0
        # for i in range(3):
        #     if idx == len(path)-1:
        #         break
        #     sum_angle += hf.angle_error(hf.angle(path[idx], path[idx+1]), prec_angle)
        #     prec_angle = hf.angle(path[idx], path[idx+1])

        if abs(angle_error) < 0.05:
            # print('rettilineo')
            #hf.plot_point_red(p, 'green')
            v_ref += 2
        elif abs(angle_error) > 0.15:
            #hf.plot_point_red(p, 'red')
            v_ref -= 1.5
            if abs(angle_error) > 0.3:
                #print('curva estrema')
                #hf.plot_point_red(p, 'blue')
                v_ref -= 1
        # else:
        #     hf.plot_point_red(p, 'orange')
        #hf.save()

        for player in sim_obs.players.keys():
            if player != self.name:
                cog2 = sim_obs.players[player].occupancy.centroid
                psi2 = sim_obs.players[player].state.psi
                if cog2.distance(Point(cog)) < 11:
                    v_ref -= 2
                    cog2 = (cog2.x, cog2.y)
                    v_cog = hf.vector(1, hf.angle(cog, cog2))
                    v_psi = hf.vector(1, psi)
                    v_psi2 = hf.vector(1, psi2)
                        

                    if v_cog@v_psi > 0.7:
                        #print('veicolo di fronte a me')
                        v_ref = 1
                        if v_psi@v_psi2 > 0.65:
                            #print(self.name + str(' in coda per ') + str(player))
                            if hf.distance(cog, cog2) < 5:
                                v_ref = -0.1
                            v_ref = 0
                        if v_psi@v_psi2 < -0.8:
                            #print('schivata')
                            angle_cog_pp = hf.angle_error(psi, hf.angle(cog, cog2))
                            if abs(angle_cog_pp) < 0.6:
                                delta_ref += 5/angle_cog_pp

                    if hf.angle_error(psi2, psi)>0:
                        #print(self.name + str(' va verso ') + player)
                        if v_cog@v_psi < 0.8 and v_cog@v_psi>-0.35:
                            # print('---------------------------')
                            #print(self.name + str(' è a lato di ') + player)
                            if v_psi@v_psi2 < 0.7 and v_psi@v_psi2 > -0.4:
                                # print(self.name + str('non è parallelo a') + player)
                                if hf.angle_error(psi, hf.angle(cog, cog2)) > 0:
                                    #print(self.name + str(' dà precedenza a destra ') + player)
                                    v_ref = 0
                                    if hf.distance(cog, cog2) < 5:
                                        v_ref = -0.1
                                if hf.angle_error(psi, hf.angle(cog, cog2)) < 0:
                                    v_ref = min(v_ref, 2)
                            #print('------------------')

        
        # p controllers
        kp_acc = 2
        if v_ref-v>0:
            kp_acc = 1
        kp_ddelta = 1.2
        kp_angerr = 3
        delta_ref = max(min(delta_ref, 1), -1)
        angle_error = max(min(angle_error, 1), -1)
        ddelta = kp_ddelta*(delta_ref - delta) + kp_angerr*(angle_error-delta)
        acc = max(min(kp_acc*(v_ref-v), 5), -8)

        if abs(ddelta)>0.5:
            ddelta = np.sign(ddelta)*0.5
        if np.abs(hf.angle(p, self.goal_point)-psi) < 0.15 and not hf.path_collision_check_safety_certificate((p, self.goal_point), self.nominal_radius, self.stat_obs['polygons']+self.stat_obs['rings']+self.stat_obs['lines']):
            acc = 4
            ddelta*=0.3
            #print('speeding up')

        #hf.save()

        return VehicleCommands(acc, ddelta)



class hf:

    @staticmethod
    def distance(point1, point2):
        return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

    @staticmethod
    def rot_mat_RW(csi):
        R = np.zeros([2,2])
        R[0,0] = np.cos(csi)
        R[0,1] = -np.sin(csi)
        R[1,0] = np.sin(csi)
        R[1,1] = np.cos(csi)
        return R.T

    @staticmethod
    def normalized_cross(vec1, vec2):
        return vec1@vec2/(np.linalg.norm(vec1)*np.linalg.norm(vec2))

    @staticmethod
    def plot_point(point):
        plt.plot(point[0], point[1], 'bo')
    
    @staticmethod
    def plot_point_red(point, color = 'red', size=2):
        plt.plot(point[0], point[1], marker=".", markersize=size, color = color)
    
    @staticmethod
    def plot_pose(point, angle, color = 'blue'):
        hf.plot_point_red(point, 'blue', 3)
        end_point = np.array([point[0], point[1]])+hf.vector(10, angle)
        end_point = (end_point[0], end_point[1])
        hf.plot_path((point, end_point), color)

    @staticmethod
    def build_box(polygon: Polygon):
        bounds = polygon.bounds
        left_down = (bounds[0], bounds[1])
        right_down = (bounds[2], bounds[1])
        right_top = (bounds[2], bounds[3])
        left_top = (bounds[0], bounds[3])
        return Polygon((left_down, right_down, right_top, left_top))

    @staticmethod
    def shoot(square_map: Polygon):
        bounds = square_map.bounds
        random_x = random.uniform(bounds[0], bounds[2])
        random_y = random.uniform(bounds[1], bounds[3])
        return (random_x, random_y)

    @staticmethod
    def neighbours(point_list, new_point: Point, n):
        neighbours_list = []
        csi = np.pi  # volume of 2d unit ball is pi*1^2
        r = 55*((1/csi)*(np.log(n)/n))**(1/2)
        # print('r is ' + str(r))
        for point in point_list:
            if hf.distance(point, new_point) < r:
                neighbours_list.append(point)
        return neighbours_list

    @staticmethod
    def generate_graph(initial_point, polygon_map: Polygon):
        graph = Graph()
        graph.E[initial_point] = []
        squared_map = hf.build_box(polygon_map)

        sequence = hf.shoot_halton_sequence(squared_map, 1500)
        for n, point in enumerate(sequence):
            if polygon_map.intersects(Point(point)):
                graph.add_point((point[0], point[1]), n+2)

        # for n in range(2, 1000):
        #     while True:
        #         new_point = hf.shoot(squared_map)
        #         if polygon_map.intersects(Point(new_point)):
        #             break
        #     graph.add_point(new_point, n)

        return graph

    
    @staticmethod
    def square_around(point, radius):
        v = [(point[0]-radius, point[1]-radius)]
        v.append((point[0]+radius, point[1]-radius))
        v.append((point[0]+radius, point[1]+radius))
        v.append((point[0]-radius, point[1]+radius))
        return Polygon(v)

    @staticmethod
    def graph_around_path(path, polygon_map: Polygon, radius, obstacles):
        graph = Graph()
        obstacles_tree = STRtree(obstacles)
        for point in path:
            square = hf.square_around(point, 9)
            for n in range(2, 8):
                while True:
                    new_point = hf.shoot(square)
                    if polygon_map.intersects(Point(new_point)):
                        if not hf.collision_point_obs(new_point, obstacles_tree, radius):
                            break
                        # valid = True
                        # for obs in obstacles:
                        #     if obs.distance(Point(new_point))<radius:
                        #         valid = False
                        #         break
                        # if valid:
                        #     break
                graph.add_point_no_collision(new_point, n, radius, obstacles)
        return graph

    @staticmethod
    def graph_around_path_hlt(path, polygon_map: Polygon, radius, obstacles, d = 15, exp = 2, N = 2):
        graph = Graph()
        n_point_seq = int(N*10)
        for point in path:
            n = 0
            square = hf.square_around(point, d)
            point_sequence = hf.shoot_halton_sequence(square, n_point_seq)
            for new_point in point_sequence:
                curr_N = N
                if polygon_map.intersects(Point(new_point)):
                    for obs in obstacles:
                        shifted = False
                        while obs.distance(Point(new_point))<radius:
                            o = obs.centroid
                            new_point = (new_point[0]-0.1*(o.x-new_point[0]), new_point[1]-0.1*(o.y-new_point[1]))
                            shifted = True
                        if shifted:
                            curr_N += exp
                            break
                    if polygon_map.intersects(Point(new_point)):
                        n += 1
                        new_point = (round(new_point[0], 4), round(new_point[1],4))
                        graph.add_point_no_collision(new_point, 200, radius, obstacles)
                        
                    if n >= curr_N:
                        break

        return graph


    @staticmethod
    def plot_graph(graph: Graph):
        for point1 in graph.E.keys():
            for point2 in graph.E[point1]:
                x_values = [point1[0], point2[0]]
                y_values = [point1[1], point2[1]]
                plt.plot(x_values, y_values, 'gray', linewidth=0.1)
    
    @staticmethod
    def plot_path(path, color='blue', width = 1):
        for idx, _ in enumerate(path[:-1]):
            point1 = path[idx]
            point2 = path[idx+1]
            x_values = [point1[0], point2[0]]
            y_values = [point1[1], point2[1]]
            plt.plot(x_values, y_values, color, linewidth = width)

    @staticmethod
    def heuristic(point1, point2) -> float:
        return hf.distance(point1, point2)

    @staticmethod
    def point_circle_collision(point, circle):
        # here circle is a tuple (center, radius)
        return hf.distance(point, circle[0]) < circle[1]

    @staticmethod
    def path_collision_check_safety_certificate(path, r, obstacles):
        collision_list = []
        safe_circles = []
        for idx, _ in enumerate(path[:-1]):
            p1 = path[idx]
            p2 = path[idx+1]
            theta = hf.angle(p1, p2)

            current_point = p1
            cycling = True
            while cycling:

                current_point_shapely = Point(current_point)
                safe_distance = min([obstacle.distance(current_point_shapely) for obstacle in obstacles])-r
                if safe_distance < 0.001:
                    collision_list.append(idx)
                    break
                else:
                    safe_circles.append((current_point, safe_distance))
                    current_point_np = np.array([current_point[0], current_point[1]])
                    next_point_np = current_point_np + hf.vector(safe_distance,theta)
                    new_point = (next_point_np[0], next_point_np[1])

                    current_point = new_point

                if hf.point_circle_collision(p2, safe_circles[-1]):
                    cycling = False

        return collision_list


    @staticmethod
    def angle(p1, p2):
        if p1[0]==p2[0]:
            return np.pi/2
        return np.arctan2(p2[1]-p1[1], p2[0]-p1[0])

    @staticmethod
    def angle_error(angle1, angle2):
        angle_error = angle1-angle2
        if angle_error < -np.pi:
            angle_error = 2*np.pi + angle_error
        if angle_error > np.pi:
            angle_error = 2*np.pi - angle_error
        return angle_error

    @staticmethod
    def vector(module:float, angle:float):
        return np.array([module*np.cos(angle), module*np.sin(angle)])

    @staticmethod
    def save():
        fig = plt.gcf()
        ax = fig.gca()
        ax.set_aspect('equal')
        plt.savefig('complete_map.png')
        #print('done!')
    
    @staticmethod
    def sample_goal(goal_polygon: Polygon, polygon_map: Polygon, radius:float, obstacles):
        square = hf.square_around((goal_polygon.centroid.x, goal_polygon.centroid.y), 1)
        while True:
            point = hf.shoot(square)
            if goal_polygon.intersects(Point(point)): #and polygon_map.intersects(Point(point)):
                valid = True
                for obs in obstacles:
                    if obs.intersects(Point(point).buffer(radius)):
                        valid = False
                        break
                if valid:
                    return point

    @staticmethod
    def plot_map_and_obs(polygon_map, stat_obs, polygon_goal):
        plt.plot(*polygon_map.exterior.xy, 'black')
        for polygon in stat_obs['polygons']:
            plt.plot(*polygon.exterior.xy, 'black')

        for ring in stat_obs['rings']:
            list_x = []
            list_y = []
            for point in ring.coords:
                list_x.append(point[0])
                list_y.append(point[1])
            plt.plot(list_x, list_y, 'black')
        plt.plot(*polygon_goal.exterior.xy, 'green')

    @staticmethod
    def plot_polygon(poly: Polygon):
        plt.plot(*poly.exterior.xy, 'green')

    @staticmethod
    def enrich(path, max_dist = 4):
        # adding points to cover holes
        new_path = path.copy()
        shift = 0
        for idx, point in enumerate(path[:-1]):
            if hf.distance(path[idx], path[idx+1])> max_dist:
                #hf.plot_point_red(hf.interpolate(point, path[idx+1]))
                new_path.insert(idx+1+shift, hf.interpolate(point, path[idx+1]))
                shift += 1
        return new_path

    def add_more_points_to_path(path,max_dist=4):
        #https://colab.research.google.com/drive/10FAZWfpz0TzWq7MdG9dbxUq_ZiDGPt_w?usp=sharing#scrollTo=ZZXBRKnsVxAL
        if len(path) == 0:
            return []

        new_path = []
        
        for i in range(0,len(path)-1):
            
            distance = np.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
            num_of_points = int(round(distance/max_dist))
            
            if num_of_points == 0:
                new_path.append(path[i])
                
            else:
                segment_x = (path[i+1][0] - path[i][0]) / num_of_points
                segment_y = (path[i+1][1] - path[i][1]) / num_of_points

                for j in range (0,num_of_points):
                    new_point = ((path[i][0] + j*segment_x),(path[i][1] + j*segment_y))
                    new_path.append(new_point)
                    #hf.plot_point_red(new_point)
        
        new_path.append(path[-1])
    
        return new_path

    @staticmethod
    def smoothing (path,weight_data=0.25,weight_smooth=0.3,tolerance=0.00001):
        # https://colab.research.google.com/drive/10FAZWfpz0TzWq7MdG9dbxUq_ZiDGPt_w?usp=sharing#scrollTo=ZZXBRKnsVxAL
    
        #smoothed_path = path.copy()

        #
        smoothed_path = [list(point) for point in path]
        #

        change = tolerance
        
        while change >= tolerance :
            change = 0.0

            for i in range (1,len(path)-1):
                
                for j in range (0,len(path[i])):
                    aux = smoothed_path[i][j]
                    smoothed_path[i][j] += weight_data * (path[i][j] - smoothed_path[i][j]) + weight_smooth * (smoothed_path[i-1][j] + smoothed_path[i+1][j] - (2.0 * smoothed_path[i][j]))
                    change += np.abs(aux - smoothed_path[i][j])
        #
        smoothed_path = [tuple(point) for point in smoothed_path]
        #
        return smoothed_path
    
    @staticmethod
    def interpolate(point1, point2):
        return (0.5*(point1[0]+point2[0]), 0.5*(point1[1]+point2[1]))

    @staticmethod
    def primes_from_2_to(n: int):
        sieve = np.ones(n // 3 + (n % 6 == 2), dtype=np.bool)
        for i in range(1, int(n ** 0.5) // 3 + 1):
            if sieve[i]:
                k = 3 * i + 1 | 1
                sieve[k * k // 3::2 * k] = False
                sieve[k * (k - 2 * (i & 1) + 4) // 3::2 * k] = False
        return np.r_[2, 3, ((3 * np.nonzero(sieve)[0][1:] + 1) | 1)]

    @staticmethod
    def van_der_corput(n_sample: int, base: int = 2):
        """ Van der Corput sequence
        
        Parameters:
        ----------
        
            n_sample :      int
                The number of element of the sequence
            
            base :          int
                The base of the sequence

        Return: sequence of Van der Corput: list (n_samples)

        """

        sequence = []
        for i in range(n_sample):
            n_th_number, denom = 0., 1.
            while i > 0:
                i, remainder = divmod(i, base)
                denom *= base
                n_th_number += remainder / denom
            sequence.append(n_th_number)

        return sequence

    @staticmethod
    def halton(n_sample: int, dim=2):
        """ Halton sequence
        
        Parameters:
        ----------
        
            dim :               int
                The dimension of the points of the sequence
            
            n_sample :          int
                The number of samples

        Return: sequence of Halton: NDarray(n_samples, n_features)

        """

        big_number = 10
        while True:
            base = hf.primes_from_2_to(big_number)[:dim]
            if len(base) >= dim:
                break
            big_number += 1000

        sample = [hf.van_der_corput(n_sample + 1, dim) for dim in base]
        sample = np.stack(sample, axis=-1)[1:]

        return np.array(sample)

    @staticmethod
    def shoot_halton_sequence(square_map: Polygon, n_samples):
        bounds = square_map.bounds
        halton_squence = hf.halton(n_samples)

        b_vec_0 = np.full((n_samples,1), bounds[0])
        b_vec_1 = np.full((n_samples,1), bounds[1])
        min_vec = np.hstack((b_vec_0, b_vec_1))

        b_vec_2 = np.full((n_samples,1), bounds[2])
        b_vec_3 = np.full((n_samples,1), bounds[3])
        max_vec = np.hstack((b_vec_2, b_vec_3))

        return min_vec + np.multiply((max_vec-min_vec),halton_squence)

    @staticmethod

    def PRM_complete(start, goal_point, polygon_map, radius, stat_obs):

        start_point = start[0]
        psi = start[1]

        # FIRST PATH
        # ---------------------------------------------------------
        graph = hf.generate_graph(start_point, polygon_map)
        graph.add_point_no_collision(goal_point, 2, radius, stat_obs['lines'])
        coin = 0
        while True:
            while True:
                path = graph.a_star(start_point, goal_point)
                collision_segments =  hf.path_collision_check_safety_certificate(path, radius, stat_obs['lines'])
                if len(collision_segments)==0:
                    break
                for i in collision_segments:
                    graph.remove_edge(path[i], path[i+1])
            if len(path) != 0:
                break
            if coin < 2:
                graph.add_200_points(polygon_map)
                coin += 1
            else:
                print('be')
                return []

        path = hf.add_more_points_to_path(path, 4) #2
 
        # SECOND PATH
        # ----------------------------------------------------
        total_static_obs = stat_obs['polygons']+stat_obs['rings']+stat_obs['lines']
        d = 7
        exp = 1
        N = 10

        while True:
            new_graph = hf.graph_around_path_hlt(path, polygon_map, radius, total_static_obs, d, exp, N)
            new_graph.add_point_no_collision(goal_point, 10, radius, total_static_obs)
            new_graph.add_point_no_collision(start_point, 10, radius, total_static_obs)
            
            while(True):
                new_path = new_graph.a_star_with_max_angle(start_point, psi, goal_point)
                valid = True
                for indx, point in enumerate(new_path[1:-1]):
                    if not polygon_map.intersects(Point(point)):
                        new_graph.remove_edge(path[indx], path[indx+1])
                        new_graph.remove_edge(path[indx-1], path[indx])
                        valid = False
                        break
                if valid:
                    break

            if len(new_path)!= 0:
                break
            else:
                #print('------------')
                d = min(d+5, 35)
                N = N*1.4
                #print('d is ' + str(d))
                # hf.plot_graph(new_graph)
                # hf.save()
                # exit()


        # print('second path found with lenght ' + str(len(new_path)))
        new_path = hf.add_more_points_to_path(new_path, 0.75)
        new_path = hf.smoothing(new_path)
        # hf.plot_path(new_path, 'orange', 1)
        # hf.save()

        return new_path

    @staticmethod
    def collision_point_obs(point, obstacles_tree, radius):
        point = Point(point)
        possible_collision = obstacles_tree.query(point)
        found_collision = False
        for collision in possible_collision:
            if collision.intersects(point.buffer(radius)):
                found_collision = True
        if not found_collision:
            return False
        else:
            return True
