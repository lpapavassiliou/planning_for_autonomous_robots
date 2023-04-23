from typing import List
from dg_commons import SE2Transform
from pdm4ar.exercises.ex06.collision_primitives import CollisionPrimitives, hf
from pdm4ar.exercises_def.ex06.structures import (
    Polygon,
    GeoPrimitive,
    Point,
    Segment,
    Circle,
    Triangle,
    Path,
)
import numpy as np
from shapely.geometry import Point as ShapelyPoint
# from shapely.geometry import LineString as ShapelySegment
# from shapely.geometry import Polygon as ShapelyPolygon
from shapely.strtree import STRtree

##############################################################################################
############################# This is a helper function. #####################################
# Feel free to use this function or not

COLLISION_PRIMITIVES = {
    Point: {
        Circle: lambda x, y: CollisionPrimitives.circle_point_collision(y, x),
        Triangle: lambda x, y: CollisionPrimitives.triangle_point_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_point_collision(y, x),
        Segment: CollisionPrimitives.point_on_segment,
    },
    Segment: {
        Circle: lambda x, y: CollisionPrimitives.circle_segment_collision(y, x),
        Triangle: lambda x, y: CollisionPrimitives.triangle_segment_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_segment_collision_aabb(y, x),
        Point: lambda x, y: CollisionPrimitives.point_on_segment(y, x)
    },
    Triangle: {
        Point: CollisionPrimitives.triangle_point_collision,
        Segment: CollisionPrimitives.triangle_segment_collision,
        Circle: lambda x, y: CollisionPrimitives.circle_triangle_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_triangle_collision(y, x),
    },
    Circle: {
        Point: CollisionPrimitives.circle_point_collision,
        Segment: CollisionPrimitives.circle_segment_collision,
        Polygon: CollisionPrimitives.circle_polygon_collision,
        Triangle: CollisionPrimitives.circle_triangle_collision,
        Circle: CollisionPrimitives.circle_circle_collision
    },
    Polygon: {
        Point: CollisionPrimitives.polygon_point_collision,
        Segment: CollisionPrimitives.polygon_segment_collision_aabb,
        Circle: lambda x, y: CollisionPrimitives.circle_polygon_collision(y, x),
        Polygon: CollisionPrimitives.polygon_polygon_collision,
        Triangle: CollisionPrimitives.polygon_triangle_collision
    },
}

def tube_collision(segment: Segment, radius: float, obstacles):
    tube = hf.build_tube(segment, radius)
    for obstacle in obstacles:
        # if not check_collision(tube[1], obstacle):
        #     return False
        for shape in tube[0]:
            if check_collision(shape, obstacle):
                return True
    return False


def check_collision(p_1: GeoPrimitive, p_2: GeoPrimitive) -> bool:
    """
    Checks collision between 2 geometric primitives
    Note that this function only uses the functions that you implemented in CollisionPrimitives class.
        Parameters:
                p_1 (GeoPrimitive): Geometric Primitive
                p_2 (GeoPrimitive): Geometric Primitive
    """
    assert type(p_1) in COLLISION_PRIMITIVES, "Collision primitive does not exist."
    assert (
        type(p_2) in COLLISION_PRIMITIVES[type(p_1)]
    ), "Collision primitive does not exist." + str(type(p_1)) + ' --- ' + str(type(p_2))

    collision_func = COLLISION_PRIMITIVES[type(p_1)][type(p_2)]

    return collision_func(p_1, p_2)


##############################################################################################


class CollisionChecker:
    """
    This class implements the collision check ability of a simple planner for a circular differential drive robot.

    Note that check_collision could be used to check collision between given GeoPrimitives
    check_collision function uses the functions that you implemented in CollisionPrimitives class.
    """

    def __init__(self):
        pass

    def path_collision_check(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:

        segments = hf.path_to_segments(t)
        collision_list = []
        for num_and_segment in enumerate(segments):
            if tube_collision(num_and_segment[1],r,obstacles):
                collision_list.append(num_and_segment[0])
        return collision_list


    def path_collision_check_occupancy_grid(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:

        segments = hf.path_to_segments(t)
        edge_size = 1
        obstacle_points = []
        for obstacle in obstacles:
            aabb = hf.form_to_aabb(obstacle)
            for i in np.arange(round(aabb.p_min.x,1), round(aabb.p_max.x,1), edge_size):
                for j in np.arange(round(aabb.p_min.y,1), round(aabb.p_max.y,1), edge_size):
                    if check_collision(Point(i,j), obstacle):
                        obstacle_points.append(Point(i,j))
        collision_list = []
        for num_and_segment in enumerate(segments):
            if tube_collision(num_and_segment[1],r, obstacle_points):
                collision_list.append(num_and_segment[0])
        return collision_list



    def path_collision_check_r_tree(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
    
        # segments = hf.path_to_segments(t)
        # squared_obstacles = hf.square_forms(obstacles)
        # for squared_obstacle in squared_obstacles:
        #     squared_obstacle = squared_obstacle.to_shapely()
        # tree = shapely.STRtree(squared_obstacles, 4)

        collision_list = []
        segments = hf.path_to_segments(t)

        shapely_obstacles = hf.to_shapely_list(obstacles)
        obstacle_tree = STRtree(shapely_obstacles)

        for num_and_segment in enumerate(segments):
            for tube_shape in hf.build_tube(num_and_segment[1], r)[0]:
                shapely_tube_shape = hf.to_shapely(tube_shape)
                query = obstacle_tree.query(shapely_tube_shape)
                for obstacle in query:
                    if shapely_tube_shape.intersects(obstacle):
                        collision_list.append(num_and_segment[0])
                        break
        return collision_list



    def collision_check_robot_frame(
        self,
        r: float,
        current_pose: SE2Transform,
        next_pose: SE2Transform,
        observed_obstacles: List[GeoPrimitive],
    ) -> bool:
        """
        Returns there exists a collision or not during the movement of a circular differential drive robot until its next pose.

            Parameters:
                    r (float): Radius of circular differential drive robot
                    current_pose (SE2Transform): Current pose of the circular differential drive robot
                    next_pose (SE2Transform): Next pose of the circular differential drive robot
                    observed_obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives in robot frame
                    Please note that only Triangle, Circle and Polygon exist in this list
        """

        # build segment in robot frame from poses
        distance = np.linalg.norm(next_pose.p-current_pose.p)
        segment = Segment(Point(0.0,0.0), Point(distance, 0.0))
        # check collision with tube_collision
        for obstacle in observed_obstacles:
            if tube_collision(segment, r, [obstacle]):
                return True
        return False



    def path_collision_check_safety_certificate(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will implement the safety certificates procedure for collision checking.
        You are free to use shapely to calculate distance between a point and a GoePrimitive.
        For more information, please check Algorithm 1 inside the following paper:
        https://journals.sagepub.com/doi/full/10.1177/0278364915625345.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """

        collision_list = []
        segments = hf.path_to_segments(t)
        shapely_obstacles = hf.to_shapely_list(obstacles)

        safe_circles = []
        for num_and_segment in enumerate(segments):
            
            theta = hf.segment_to_angle(num_and_segment[1])
            initial_point = num_and_segment[1].p1
            final_point = num_and_segment[1].p2

            current_point = initial_point
            cycling = True
            while cycling:
                
                current_point_shapely = hf.to_shapely(current_point)
                safe_distance = min([obstacle.distance(current_point_shapely) for obstacle in shapely_obstacles])-r
                if safe_distance < 0.0001:
                    collision_list.append(num_and_segment[0])
                    break
                else:
                    safe_circles.append(Circle(current_point, safe_distance))
                    current_point_np = hf.point_to_array(current_point)
                    current_point_np = current_point_np + hf.vector(safe_distance,theta)
                    new_point = hf.array_to_point(current_point_np)

                    current_point = new_point

                if check_collision(safe_circles[-1], final_point):
                    cycling = False

        return collision_list
