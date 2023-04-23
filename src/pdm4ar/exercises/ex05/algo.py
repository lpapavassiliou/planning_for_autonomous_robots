from typing import Sequence

from dg_commons import SE2Transform

from pdm4ar.exercises.ex05.structures import *
from pdm4ar.exercises_def.ex05.utils import extract_path_points


class PathPlanner(ABC):
    @abstractmethod
    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        pass


class Dubins(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> List[SE2Transform]:
        """ Generates an optimal Dubins path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a List[SE2Transform] of configurations in the optimal path the car needs to follow
        """
        path = calculate_dubins_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list


class ReedsShepp(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        """ Generates a Reeds-Shepp *inspired* optimal path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a List[SE2Transform] of configurations in the optimal path the car needs to follow 
        """
        path = calculate_reeds_shepp_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list



# ----------- defined by me ---------------------


def calculate_car_turning_radius(wheel_base: float, max_steering_angle: float) -> DubinsParam:
    # TODO implement here your solution
    min_radius = wheel_base/np.tan(max_steering_angle)
    return DubinsParam(min_radius)


def unitaryVector(theta):
    # given theta, returns unitary vector in that direction.
    return np.array([np.cos(theta), np.sin(theta)])

def angleGivenPoints(x, y):
    # given two 2D points x, y, returns angle of the vector (x->y)
    return( np.arctan2( (y[1]-x[1]), (y[0]-x[0])) )

def path_lenght(path : Path) -> float:
    lenght = 0
    for segment in path:
        lenght += segment.length
    return lenght

def vector(module, angle):
    return np.array([module*np.cos(angle), module*np.sin(angle)])

def arc_RIGHT(start_config: SE2Transform, end_config: SE2Transform, circle: Curve):
    theta = angleGivenPoints(circle.center.p, start_config.p) - angleGivenPoints(circle.center.p, end_config.p)
    return Curve(start_config, end_config, circle.center, circle.radius, DubinsSegmentType.RIGHT, theta)

def arc_LEFT(start_config: SE2Transform, end_config: SE2Transform, circle: Curve):
    theta = angleGivenPoints(circle.center.p, end_config.p) -angleGivenPoints(circle.center.p, start_config.p)
    return Curve(start_config, end_config, circle.center, circle.radius, DubinsSegmentType.LEFT, theta)

def mid_circles(circle_1 : Curve, circle_2: Curve):
    
        # find centers of the two mid circles A, B
        l = np.linalg.norm(circle_1.center.p - circle_2.center.p)
        d = np.sqrt((2*circle_1.radius)**2 - (l/2)**2)
        theta_centers = angleGivenPoints(circle_1.center.p, circle_2.center.p)
        center_A_p = circle_1.center.p + vector(l/2, theta_centers) + vector(d, theta_centers+np.pi/2)
        center_B_p = circle_1.center.p + vector(l/2, theta_centers) + vector(d, theta_centers+np.pi/2)
        center_A = SE2Transform(center_A_p, 0)
        center_B = SE2Transform(center_B_p, 0)

        # creating mid circles A and B. Valid configurations are simply the highest points of the circles with theta = 0
        valid_config_A = SE2Transform(center_A_p + vector(circle_1.radius, np.pi/2), 0)
        valid_config_B = SE2Transform(center_B_p + vector(circle_1.radius, np.pi/2), 0)
        circle_A = Curve.create_circle(center=center_A, config_on_circle = valid_config_A, radius = circle_1.radius, curve_type = DubinsSegmentType.LEFT)
        circle_B = Curve.create_circle(center=center_B, config_on_circle = valid_config_B, radius = circle_1.radius, curve_type = DubinsSegmentType.LEFT)

        return circle_A, circle_B


def tangent_configurations_RLR(circle_1, circle_2, circle_A, circle_B):
        # returns configurations A1, A2, B1, B2

        theta_A1 = angleGivenPoints(circle_A.center.p, circle_1.center.p)
        theta_A2 = angleGivenPoints(circle_A.center.p, circle_2.center.p)
        A1_p = circle_A.center.p + vector(circle_A.radius, theta_A1)
        A2_p = circle_A.center.p + vector(circle_A.radius, theta_A2)

        theta_B1 = angleGivenPoints(circle_B.center.p, circle_1.center.p)
        theta_B2 = angleGivenPoints(circle_B.center.p, circle_2.center.p)
        B1_p = circle_B.center.p + vector(circle_B.radius, theta_B1)
        B2_p = circle_B.center.p + vector(circle_B.radius, theta_B2)

        return(SE2Transform(A1_p, theta_A1 + np.pi/2), SE2Transform(A2_p, theta_A2 + np.pi/2), SE2Transform(B1_p, theta_B1 + np.pi/2), SE2Transform(B2_p, theta_B2 + np.pi/2))

def tangent_configurations_LRL(circle_1, circle_2, circle_A, circle_B):
        # returns configurations A1, A2, B1, B2

        theta_A1 = angleGivenPoints(circle_A.center.p, circle_1.center.p)
        theta_A2 = angleGivenPoints(circle_A.center.p, circle_2.center.p)
        A1_p = circle_A.center.p + vector(circle_A.radius, theta_A1)
        A2_p = circle_A.center.p + vector(circle_A.radius, theta_A2)

        theta_B1 = angleGivenPoints(circle_B.center.p, circle_1.center.p)
        theta_B2 = angleGivenPoints(circle_B.center.p, circle_2.center.p)
        B1_p = circle_B.center.p + vector(circle_B.radius, theta_B1)
        B2_p = circle_B.center.p + vector(circle_B.radius, theta_B2)

        return(SE2Transform(A1_p, theta_A1 - np.pi/2), SE2Transform(A2_p, theta_A2 - np.pi/2), SE2Transform(B1_p, theta_B1 - np.pi/2), SE2Transform(B2_p, theta_B2 - np.pi/2))


def invert_path(path:Path):
    # here we switch start end end configuration of every segment and we 
    # reverse the order of the segments in the path list
    new_path = []
    for segment in path:
        segment.gear = Gear.REVERSE
        temp = segment.end_config
        segment.end_config = segment.start_config
        segment.start_config = temp
        new_path.insert(0, segment)
    return new_path

def calculate_turning_circles(current_config: SE2Transform, radius: float) -> TurningCircle:
    # TODO implement here your solution

    # get coordinates of the centers
    center_L = current_config.p + radius*unitaryVector(current_config.theta + np.pi/2)
    center_R = current_config.p - radius*unitaryVector(current_config.theta + np.pi/2)

    # build SE2 of the centers
    center_L = SE2Transform(center_L, 0)
    center_R = SE2Transform(center_R, 0)

    # build circles
    left_circle = Curve.create_circle(center= center_L, config_on_circle=current_config,
                                       radius = radius, curve_type=DubinsSegmentType.LEFT)
    right_circle = Curve.create_circle(center= center_R, config_on_circle=current_config,
                                       radius = radius, curve_type=DubinsSegmentType.RIGHT)
    return TurningCircle(left_circle, right_circle)


def calculate_tangent_btw_circles(circle_start: Curve, circle_end: Curve) -> Line:
    # TODO implement here your solution

    # If the two cirlces have same sense of travel (R,R or L,L) there is an only viable tangent, 
    # which is parallel to the line that connects the two centers.
    # If the two cirlces have different sense of travel (R,L or L,R), there is an only viable tangent, 
    # which is can be recovered from the line that connects the centers adding an inclination.

    # if circles are inside each other and they are LR or RL there is no cross tangent
    if np.linalg.norm(circle_start.center.p - circle_end.center.p) < circle_start.radius + circle_end.radius and circle_start.type != circle_end.type:
        return []
    
    if circle_start.type == circle_end.type:
        
        # signLR is 1 if the circles are RR and -1 if they are LL.
        sign_multiplicator = 1
        if circle_start.type == DubinsSegmentType.LEFT:
            sign_multiplicator = -1

        # get angle of center line
        theta_center_line = angleGivenPoints(circle_start.center.p, circle_end.center.p)

        # get start and end configurations of the tangent
        point_start = circle_start.center.p + circle_start.radius*unitaryVector(theta_center_line + sign_multiplicator*np.pi/2)
        point_end   =  circle_end.center.p  + circle_end.radius*unitaryVector(theta_center_line + sign_multiplicator*np.pi/2)
        start_config = SE2Transform(point_start, theta_center_line)
        end_config = SE2Transform(point_end, theta_center_line)

        # build the tangent
        tangent = Line(start_config, end_config)
        return [tangent]
    
    else:
        
        # signLR is 1 if the circles are RL and -1 if they are LR.
        sign_multiplicator = 1
        if circle_start.type == DubinsSegmentType.LEFT:
            sign_multiplicator = -1
        
        # get angle of center line
        theta_center_line = angleGivenPoints(circle_start.center.p, circle_end.center.p)

        # get lenght of center line
        lenght_center_line = np.linalg.norm(circle_start.center.p-circle_end.center.p)

        # get start and end configuration of the tangent

        alpha = np.arccos( circle_start.radius/(0.5*lenght_center_line) )
        point_start = circle_start.center.p + circle_start.radius*unitaryVector(theta_center_line + sign_multiplicator*alpha)
        point_end =   circle_end.center.p   -  circle_end.radius*unitaryVector(theta_center_line + sign_multiplicator*alpha)
        theta = theta_center_line + sign_multiplicator*alpha - sign_multiplicator*np.pi/2
        start_config = SE2Transform(point_start, theta)
        end_config = SE2Transform(point_end, theta)


        # build the tangent
        tangent = Line(start_config, end_config)
        return [tangent]



def calculate_dubins_path(start_config: SE2Transform, end_config: SE2Transform, radius: float, triple_curves_needed: bool = True) -> Path:
    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins path!

    # ------------ draw circles for start and end ------------
    circles_start = calculate_turning_circles(start_config, radius)
    circles_end = calculate_turning_circles(end_config, radius)


    # ------------ find candidate paths ----------------
    candidates = []
    
    # RSR
    #################################################################

    circle_1 = circles_start.right
    circle_2 = circles_end.right
    tangent_list = calculate_tangent_btw_circles(circle_1, circle_2)
    tangent = tangent_list[0] # we know there's only 1 tangent
    
    if np.array_equal(circle_1.center.p,circle_2.center.p):
    # if circles are coincident (and they are RR, but this has been checked) there is a path on the circle
        arc = arc_RIGHT(start_config, end_config, circle_1)
        null_line = Line(end_config, end_config)
        null_arc = arc_RIGHT(end_config, end_config, circle_1)
        candidates.append([arc, null_line, null_arc])

    else:
        tangent_list = calculate_tangent_btw_circles(circle_1, circle_2)
        tangent = tangent_list[0] # we know there's only 1 tangent

        arc_1 = arc_RIGHT(start_config, tangent.start_config, circle_1)
        arc_2 = arc_RIGHT(tangent.end_config, end_config, circle_2)
        candidates.append([arc_1, tangent, arc_2])


    # LSL
    #################################################################
    circle_1 = circles_start.left
    circle_2 = circles_end.left

    if np.array_equal(circle_1.center.p, circle_2.center.p):
        arc = arc_LEFT(start_config, end_config, circle_1)
        null_line = Line(end_config, end_config)
        null_arc = arc_LEFT(end_config, end_config, circle_1)
        candidates.append([arc, null_line, null_arc])

    else:

        tangent_list = calculate_tangent_btw_circles(circle_1, circle_2)
        tangent = tangent_list[0]

        arc_1 = arc_LEFT(start_config, tangent.start_config, circle_1)
        arc_2 = arc_LEFT(tangent.end_config, end_config, circle_2)

        candidates.append([arc_1, tangent, arc_2])


    # RSL
    #################################################################
    circle_1 = circles_start.right
    circle_2 = circles_end.left

    tangent_list = calculate_tangent_btw_circles(circle_1, circle_2)
    if len(tangent_list) != 0:
        tangent = tangent_list[0]

        arc_1 = arc_RIGHT(start_config, tangent.start_config, circle_1)
        arc_2 = arc_LEFT(tangent.end_config, end_config, circle_2)

        candidates.append([arc_1, tangent, arc_2])


    # LSR
    #################################################################
    circle_1 = circles_start.left
    circle_2 = circles_end.right

    tangent_list = calculate_tangent_btw_circles(circle_1, circle_2)
    if len(tangent_list) != 0:
        tangent = tangent_list[0]
        arc_1 = arc_LEFT(start_config, tangent.start_config, circle_1)
        arc_2 = arc_RIGHT(tangent.end_config, end_config, circle_2)
        candidates.append([arc_1, tangent, arc_2])

    if triple_curves_needed == True:
        # RLR (0 or 2 paths)
        #################################################################
        circle_1 = circles_start.right
        circle_2 = circles_end.right

        # If the distance between circles is higher than 4*radius, we can't connect the two circles with another circles.
        if np.linalg.norm(circle_1.center.p - circle_2.center.p) <= 4*circle_1.radius:
            circle_A, circle_B = mid_circles(circle_1, circle_2)

            config_A1, config_A2, config_B1, config_B2 = tangent_configurations_RLR(circle_1, circle_2, circle_A, circle_B)

            # building path A
            arc_1 = arc_RIGHT(start_config, config_A1, circle_1)
            arc_A = arc_LEFT(config_A1, config_A2, circle_A)
            arc_2 = arc_RIGHT(config_A2, end_config, circle_2)
            candidates.append([arc_1, arc_A, arc_2])

            # building path B
            arc_1 = arc_RIGHT(start_config, config_B1, circle_1)
            arc_B = arc_LEFT(config_B1, config_B2, circle_B)
            arc_2 = arc_RIGHT(config_B2, end_config, circle_2)
            candidates.append([arc_1, arc_B, arc_2])
        

        # LRL (0 or 2 paths)
        #################################################################
        # If the distance between circles is higher than 4*radius, we can't connect the two circles with another circles.

        circle_1 = circles_start.left
        circle_2 = circles_end.left

        if np.linalg.norm(circle_1.center.p - circle_2.center.p) <= 4*circle_1.radius:

            circle_A, circle_B = mid_circles(circle_1, circle_2)

            config_A1, config_A2, config_B1, config_B2 = tangent_configurations_LRL(circle_1, circle_2, circle_A, circle_B)

            # building path A
            arc_1 = arc_LEFT(start_config, config_A1, circle_1)
            arc_A = arc_RIGHT(config_A1, config_A2, circle_A)
            arc_2 = arc_LEFT(config_A2, end_config, circle_2)
            candidates.append([arc_1, arc_A, arc_2])

            # building path B
            arc_1 = arc_LEFT(start_config, config_B1, circle_1)
            arc_B = arc_RIGHT(config_B1, config_B2, circle_B)
            arc_2 = arc_LEFT(config_B2, end_config, circle_2)
            candidates.append([arc_1, arc_B, arc_2])

    # ------------ choose best path ----------------

    best_path = candidates[0]
    min_lenght = path_lenght(candidates[0])

    for path in candidates:
        if path_lenght(path) < min_lenght:
            min_lenght = path_lenght(path)
            best_path = path
    
    return best_path # e.g., [Curve(), Line(),..]



def calculate_reeds_shepp_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:
    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins/reeds path!
    forward_path = calculate_dubins_path(start_config, end_config, radius)
    backward_path = calculate_dubins_path(end_config, start_config, radius, triple_curves_needed=False)
    backward_path = invert_path(backward_path)
    if path_lenght(forward_path)< path_lenght(backward_path):
        return forward_path
    return backward_path
