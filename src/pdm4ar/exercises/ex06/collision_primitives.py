from pdm4ar.exercises_def.ex06.structures import *
import numpy as np
import triangle
from typing import List
from dg_commons import SE2Transform
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import LineString as ShapelySegment
from shapely.geometry import Polygon as ShapelyPolygon


class hf:            # helping functions defined by me

    @staticmethod
    def point_to_array(p: Point):
        return(np.array([p.x, p.y]))

    @staticmethod
    def array_to_point(array):
        return(Point(array[0], array[1]))

    @staticmethod
    def distance(p1: Point, p2: Point):
        return np.linalg.norm(hf.point_to_array(p1) - hf.point_to_array(p2))

    @staticmethod
    def angle_np(p1, p2):
        vector_12 = p2-p1
        return np.arctan2(vector_12[1], vector_12[0])

    @staticmethod
    def vector(module:float, angle:float):
        return np.array([module*np.cos(angle), module*np.sin(angle)])

    @staticmethod
    def build_tube(s: Segment, radius: float):
        # given a circle (only its radius matters) and a segment, returns the geometric figure obtained through 
        # sliding the circle on the segment, via a list of:
        # - start circle
        # - end circle
        # - rectangle

        circle_1 = Circle(s.p1, radius)
        circle_2 = Circle(s.p2, radius)

        p1_np = hf.point_to_array(s.p1)
        p2_np = hf.point_to_array(s.p2)
        theta_c = hf.angle_np(p1_np, p2_np)
        theta = theta_c + np.pi/2

        p_1u = p1_np + hf.vector(radius, theta)
        p_1d = p1_np - hf.vector(radius, theta)
        p_2u = p2_np + hf.vector(radius, theta)
        p_2d = p2_np - hf.vector(radius, theta)

        q_1u = hf.array_to_point(p_1u - hf.vector(radius, theta_c))
        q_1d = hf.array_to_point(p_1d - hf.vector(radius, theta_c))
        q_2u = hf.array_to_point(p_2u + hf.vector(radius, theta_c))
        q_2d = hf.array_to_point(p_2d + hf.vector(radius, theta_c))

        p_1u = hf.array_to_point(p_1u)
        p_1d = hf.array_to_point(p_1d)
        p_2u = hf.array_to_point(p_2u)
        p_2d = hf.array_to_point(p_2d)

        rectangle =     Polygon([p_1u, p_1d, p_2d, p_2u])
        big_rectangle = Polygon([q_1u, q_1d, q_2d, q_2u])

        return [[rectangle, circle_1, circle_2], big_rectangle]

    @staticmethod
    def segment_to_angle(segment: Segment):
        p1 = hf.point_to_array(segment.p1)
        p2 = hf.point_to_array(segment.p2)
        return hf.angle_np(p1,p2)

    @staticmethod
    def form_to_aabb(form):
        boundaries = form.get_boundaries()
        return AABB(boundaries[0],boundaries[1])

    @staticmethod
    def box_collision(form1, form2):
        aabb_1 = hf.form_to_aabb(form1)
        aabb_2 = hf.form_to_aabb(form2)
        return CollisionPrimitives.aabb_aabb_collision(aabb_1, aabb_2)
        
    @staticmethod
    def square_forms(forms):
        return [hf.form_to_aabb(form) for form in forms]

    # @staticmethod
    # def big_square_rounded(squares: List[AABB]) -> AABB:
    #     min_x_list = []
    #     min_y_list = []
    #     max_x_list = []
    #     max_y_list = []
    #     for square in squares:
    #         min_x_list.append(square.p_min.x)
    #         min_y_list.append(square.p_min.y)
    #         max_x_list.append(square.p_max.x)
    #         max_y_list.append(square.p_max.y)
    #     x_min = min(min_x_list)
    #     y_min = min(min_y_list)
    #     x_max = max(max_x_list)
    #     y_max = max(max_y_list)
    #     p_min = Point(np.floor(x_min),np.floor(y_min))
    #     p_max = Point(np.ceil(x_max), np.ceil(y_max))
    #     return AABB(p_min, p_max)

    @staticmethod
    def path_to_segments(t:Path):
        segments = []
        for num_and_point in enumerate(t.waypoints):
            if num_and_point[0] != len(t.waypoints)-1:
                segments.append(Segment(t.waypoints[num_and_point[0]], t.waypoints[num_and_point[0]+1]))
        return segments

    @staticmethod
    def get_line(segment: Segment):
        # if segment is vertical, returns m = inf and q = 0
        if segment.p2.x == segment.p1.x:
            return np.inf, 0
        m = (segment.p2.y-segment.p1.y)/(segment.p2.x-segment.p1.x)
        q = segment.p1.y - m*segment.p1.x
        return m,q

    @staticmethod
    def edges(shape):
        if isinstance(shape, Triangle):
           return([Segment(shape.v1, shape.v2), Segment(shape.v2, shape.v3), Segment(shape.v3, shape.v1)])
        if isinstance(shape, Polygon):
            edges = []
            for i in range(len(shape.vertices)-1):
                edges.append(Segment(shape.vertices[i], shape.vertices[i+1]))
            edges.append(Segment(shape.vertices[-1],shape.vertices[0]))
            return(edges)

    # @staticmethod
    # def point_to_I_frame(point:Point, current_frame: SE2Transform):
    #     point = np.array([point.x, point.y])
    #     R = np.array([[np.cos(current_frame.theta), -np.sin(current_frame.theta)], [np.sin(current_frame.theta), np.cos(current_frame.theta)]])
    #     t = current_frame.p
    #     new_point = R@point+t
    #     return(Point(new_point[0], new_point[1]))

    # @staticmethod
    # def to_I_frame(shape, current_frame: SE2Transform):
    #     if isinstance(shape, Point):
    #         point = np.array([shape.x, shape.y])
    #         R = np.array([[np.cos(current_frame.theta), -np.sin(current_frame.theta)], [np.sin(current_frame.theta), np.cos(current_frame.theta)]])
    #         t = current_frame.p
    #         new_point = R@point+t
    #         return(Point(new_point[0], new_point[1]))
    #     if isinstance(shape, Polygon):
    #         new_vertices = []
    #         for v in shape.vertices:
    #             new_vertices.append(hf.to_I_frame(v,current_frame))
    #         return(Polygon(new_vertices))
    #     if isinstance(shape, Triangle):
    #         v1_new = hf.to_I_frame(shape.v1,current_frame)
    #         v2_new = hf.to_I_frame(shape.v2,current_frame)
    #         v3_new = hf.to_I_frame(shape.v3,current_frame)
    #         return(Triangle(v1_new, v2_new, v3_new))
    #     if isinstance(shape, Circle):
    #         new_center = hf.to_I_frame(shape.center, current_frame)
    #         return(Circle(new_center,shape.radius))

    @staticmethod
    def to_shapely(shape):
        if isinstance(shape, Point):
            return ShapelyPoint(shape.x, shape.y)
        if isinstance(shape, Segment):
            return ShapelySegment([(shape.p1.x, shape.p1.y), (shape.p2.x, shape.p2.y)])
        if isinstance(shape, Polygon):
            point_list = [[vertex.x, vertex.y] for vertex in shape.vertices]
            return ShapelyPolygon(point_list)
        if isinstance(shape, Triangle):
            return ShapelyPolygon([[shape.v1.x, shape.v1.y], [shape.v2.x, shape.v2.y], [shape.v3.x, shape.v3.y]])
        if isinstance(shape, Circle):
            return ShapelyPoint(shape.center.x, shape.center.y).buffer(shape.radius)

    # @staticmethod
    # def to_our(shape):
    #     if isinstance(shape, ShapelyPoint):
    #         return Point(shape.x, shape.y)
    #     if isinstance(shape, ShapelySegment):
    #         boundaries = shape.bounds
    #         return Segment([(boundaries(0), boundaries(1)), (boundaries(2),boundaries(3))])
    #     if isinstance(shape, ShapelyPolygon):
    #         point_list = [[vertex.x, vertex.y] for vertex in shape.vertices]
    #         return Polygon(point_list)


    @staticmethod
    def to_shapely_list(shape_list):
        shapely_list = []
        for shape in shape_list:
            shapely_list.append(hf.to_shapely(shape))
        return shapely_list
            
        
class CollisionPrimitives:
    """ 
    Class of collusion primitives
    """

    @staticmethod
    def point_on_segment(point:Point, segment:Segment):
        # returns true if a point is on a segment
        # idea: if the two distances are equal to the line's length, the point is on the line
        d1 = hf.distance(point, segment.p1)
        d2 = hf.distance(point, segment.p2)
        lineLen = hf.distance(segment.p1, segment.p2)
        if d1+d2 >= lineLen-0.1 and d1+d2 <= lineLen+0.1:
            return True
        return False


    @staticmethod
    def circle_point_collision(c: Circle, p: Point) -> bool:
        if hf.distance(p, c.center) > c.radius:
            return False
        return True

    @staticmethod
    def segments_collision(a: Segment, b: Segment) -> bool:
        # idea: if the intersection of the segments' lines is on one of the lines, then they collide

        m_a, q_a = hf.get_line(a)
        m_b, q_b = hf.get_line(b)

        # particular case 1: lines are parallel
        if round(m_a,4) == round(m_b,4):
            return CollisionPrimitives.point_on_segment(a.p1, b) or CollisionPrimitives.point_on_segment(a.p2, b)

        # particular case 2: only one segment is vertical
        if m_a == np.inf:
            intersection_point = Point(a.p1.x, m_b*a.p1.x+q_b)
            if CollisionPrimitives.point_on_segment(intersection_point, a):
                if CollisionPrimitives.point_on_segment(intersection_point, b):
                    return True
        if m_b == np.inf:
            intersection_point = Point(b.p1.x, m_a*b.p1.x+q_a)
            if CollisionPrimitives.point_on_segment(intersection_point, a):
                if CollisionPrimitives.point_on_segment(intersection_point, b):
                    return True

        # general case
        x = (q_b-q_a)/(m_a-m_b)
        y = m_a*x + q_a
        intersection_point = Point(x, y)
        if CollisionPrimitives.point_on_segment(intersection_point, a):
            if CollisionPrimitives.point_on_segment(intersection_point, b):
                    return True
        return False


    @staticmethod
    def triangle_point_collision(t: Triangle, p: Point) -> bool:

        areaOrig = np.abs( (t.v2.x-t.v1.x)*(t.v3.y-t.v1.y) - (t.v3.x-t.v1.x)*(t.v2.y-t.v1.y) )
        area1 = np.abs( (t.v1.x-p.x)*(t.v2.y-p.y) - (t.v2.x-p.x)*(t.v1.y-p.y) )
        area2 = np.abs( (t.v2.x-p.x)*(t.v3.y-p.y) - (t.v3.x-p.x)*(t.v2.y-p.y) )
        area3 = np.abs( (t.v3.x-p.x)*(t.v1.y-p.y) - (t.v1.x-p.x)*(t.v3.y-p.y) )
        if round(area1 + area2 + area3, 4) == round(areaOrig,4):
            return True
        return False


    @staticmethod
    def aabb_aabb_collision(aabb_1: AABB, aabb_2: AABB):
        if aabb_2.p_max.x < aabb_1.p_min.x or aabb_2.p_min.x > aabb_1.p_max.x:
            return False
        if aabb_2.p_max.y < aabb_1.p_min.y or aabb_2.p_min.y > aabb_1.p_max.y:
            return False
        return True


    @staticmethod
    def polygon_point_collision(poly: Polygon, p: Point) -> bool:

        # divide poly into triangles
        vertices = []
        for vertex in poly.vertices:
            vertices.append(hf.point_to_array(vertex))
        t_dict = triangle.triangulate({'vertices': vertices})
        triples_list = t_dict['triangles'].tolist()

        # create Triangle type triangles and check collision
        for triple in triples_list:
            vertices_nparr = np.array(vertices)[np.array(triple).astype(int)]
            t = Triangle(hf.array_to_point(vertices_nparr[0]), hf.array_to_point(vertices_nparr[1]), hf.array_to_point(vertices_nparr[2]))
            if CollisionPrimitives.triangle_point_collision(t, p):
                return True
        return False


    @staticmethod
    def circle_segment_collision(c: Circle, segment: Segment) -> bool:

        inside1 = CollisionPrimitives.circle_point_collision(c, segment.p1)
        inside2 = CollisionPrimitives.circle_point_collision(c, segment.p2)
        if inside1 or inside2:
            return True
        len = hf.distance(segment.p1, segment.p2)
        dot = (((c.center.x - segment.p1.x)*(segment.p2.x-segment.p1.x)) + ((c.center.y-segment.p1.y)*(segment.p2.y-segment.p1.y)))/len**2
        closest_x = segment.p1.x + (dot*(segment.p2.x-segment.p1.x))
        closest_y = segment.p1.y + (dot*(segment.p2.y-segment.p1.y))
        closest_point = Point(closest_x, closest_y)
        on_segment = CollisionPrimitives.point_on_segment(closest_point, segment)
        if not on_segment:
            return False
        if hf.distance(closest_point, c.center) <= c.radius:
            return True
        return False


    @staticmethod
    def triangle_segment_collision(t: Triangle, segment: Segment) -> bool:

        # check if segment boundaries are inside the triangle
        if CollisionPrimitives.triangle_point_collision(t, segment.p1):
                return True
        if CollisionPrimitives.triangle_point_collision(t, segment.p2):
                return True

        # check if segment instersects any triangle's edge
        for edge in hf.edges(t):
            if CollisionPrimitives.segments_collision(segment, edge):
                return True
        return False


    @staticmethod
    def polygon_segment_collision(p: Polygon, segment: Segment) -> bool:

        # case 1: one of the segment extrema is inside the polygon
        if CollisionPrimitives.polygon_point_collision(p, segment.p1):
            return True
        if CollisionPrimitives.polygon_point_collision(p, segment.p2):
            return True

        # case 2: segment crosses the polygon
        for edge in  hf.edges(p):
            if CollisionPrimitives.segments_collision(segment, edge):
                return True
        return False


    @staticmethod
    def polygon_segment_collision_aabb(p: Polygon, segment: Segment) -> bool:
        # a first check is done with the aabb boxes of polygon and segment. If test is negative,
        # then we execute polygon_segment_collision code.
        if not hf.box_collision(p, segment):
            return False
        return(CollisionPrimitives.polygon_segment_collision(p, segment))

    @staticmethod
    def polygon_polygon_collision(p1: Polygon, p2: Polygon):

        # aabb check
        if not hf.box_collision(p1, p2):
            return False
        # case 1: polygon1 is inside polygon2
        if not CollisionPrimitives.polygon_point_collision(p1, p2.vertices[0]):
            return False
        # case 2: polygon2 is inside polygon1
        if not CollisionPrimitives.polygon_point_collision(p2, p1.vertices[0]):
            return False
        # general case: segments collision
        for edge in hf.edges(p2):
            if CollisionPrimitives.polygon_segment_collision(p1, edge):
                return True
        return False

    @staticmethod
    def polygon_triangle_collision(p: Polygon, t: Triangle):

        # aabb check
        if not hf.box_collision(p, t):
            return False
        # case 1: triangle is inside polygon
        if not CollisionPrimitives.polygon_point_collision(p, t.v1):
            return False
        # case 2: polygon2 is inside polygon1
        if not CollisionPrimitives.triangle_point_collision(t, p.vertices[0]):
            return False
        # general case: segments collision
        for edge in hf.edges(t):
            if CollisionPrimitives.polygon_segment_collision(p, edge):
                return True
        return False

    # @staticmethod
    # def triangle_triangle_collision(t1: Triangle, t2: Triangle) -> bool:
        
    #     # aabb collision check
    #     t1_boundaries = t1.get_boundaries()
    #     aabb_1 = AABB(t1_boundaries[0], t1_boundaries[1])
    #     t2_boundaries = t2.get_boundaries()
    #     aabb_2 = AABB(t2_boundaries[0], t2_boundaries[1])
    #     if not CollisionPrimitives.aabb_aabb_collision(aabb_1, aabb_2):
    #         return False
    #     # case 1 [inside] -> check if one of the verteces of t2 collides with t1
    #     for vertex in [t2.v1, t2.v2, t2.v3]:
    #         if CollisionPrimitives.triangle_point_collision(t1, vertex):
    #             return True
    #     # case 2 [segment collision]
    #     for edge in t2.edges():
    #         if CollisionPrimitives.triangle_segment_collision(t1, edge):
    #             return True
    #     return False


    @staticmethod
    def circle_polygon_collision(c: Circle, p: Polygon) -> bool:

        # aabb check
        if not hf.box_collision(p, c):
            return False
        # case 1: circle inside polygon
        if CollisionPrimitives.polygon_point_collision(p, c.center):
            return True
        # case 2: polygon inside circle
        if CollisionPrimitives.circle_point_collision(c, p.vertices[0]):
            return True
        # general case: collision edge-circle
        for edge in hf.edges(p):
            if CollisionPrimitives.circle_segment_collision(c, edge):
                return True
        return False
    
    @staticmethod
    def circle_triangle_collision(c: Circle, t: Triangle) -> bool:

        # aabb collision check
        # aabb check
        if not hf.box_collision(c, t):
            return False
        # case 1: circle inside triangle
        if CollisionPrimitives.triangle_point_collision(t, c.center):
            return True
        # case 2: triangle inside circle
        if CollisionPrimitives.circle_point_collision(c, t.v1):
            return True
        # general case: collision edge-circle
        for edge in hf.edges(t):
            if CollisionPrimitives.circle_segment_collision(c, edge):
                return True
        return False

    @staticmethod
    def circle_circle_collision(c1: Circle, c2: Circle) -> bool:
        return hf.distance(c1.center, c2.center) <= c1.radius+c2.radius

    @staticmethod
    def _poly_to_aabb(g: Polygon) -> AABB:
        # todo feel free to implement functions that upper-bound a shape with an
        #  AABB or simpler shapes for faster collision checks
        return AABB(p_min=Point(0, 0), p_max=Point(1, 1))
    

