"""bug_0_algo controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS
import math
from sympy import Polygon, Point, Ellipse
import pickle

# TREE = 

goals = [(9,20), (20, 6)]*3


def calc_dir(p1, p2):
    x = p1[0]-p2[0]
    y = p1[1]-p2[1]
    return math.atan2(y, x)
    
    
def gen_line(p1, p2):
    """
    Line ax + by + c = 0
    :return: {"a": a, "b": b, "c": c}
    """
    a = - p1[1] + p2[1]
    b = p1[0] - p2[0]
    c = - b * p1[1] - a * p1[0]
    return {"a": a, "b": b, "c": c}


def calc_dist(p1, p2):
    """
    Calculates distance B/W P1, P2
    :return: distance: int
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def calc_poly_dist(poly, p):
    """
    :param p: 0-->p1, 1-->p2
    :param poly: Polygon represented by the list of tuple(x, y) of vertices.
    :return: Returns distance b/w the point and the give polygon.
    """
    v = map(Point, poly)
    poly = Polygon(*v)
    pnt = p

    return poly.distance(Point(pnt[0], pnt[1])) * (int(poly.encloses_point(Point(pnt[0], pnt[1]))) * (-2) + 1)


def calc_poly_tang(poly, p):
    """
    :param p: 0-->p1, 1-->p2
    :param poly: Polygon represented by the list of tuples(x, y) of vertices.
    :return: Returns numpy vector pointing the polygon from the point.
    """
    v = map(Point, poly)
    poly = Polygon(*v)
    pnt = p

    rad = poly.distance(Point(pnt))
    circ = Ellipse(Point(pnt), rad, rad)
    cls_pnt = poly.intersection(circ)
    x_tang = cls_pnt[0][0] - pnt[0]
    y_tang = cls_pnt[0][1] - pnt[1]
    return math.atan2(y_tang, x_tang)


def poly_intersection(poly1, poly2):
    """
    Finds intersections b/w two pol
    :param poly1: Polygon represented by the list of tuples(x, y) of vertices.
    :param poly2: Polygon represented by the list of tuples(x, y) of vertices.
    :return: List of points of intersections b/w poly1 and poly2
    """
    v1 = map(Point, poly1)
    v2 = map(Point, poly2)

    poly1 = Polygon(*v1)
    poly2 = Polygon(*v2)

    ret = list(map(list, poly1.intersection(poly2)))
    return ret


def move(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    goal_poly = [(0.5, 1.0), (0.5, 0.6), (0.1, 0.6), (0.1, 1.0)]
    tree_poly = [(0.4, 0.9), (0.4, 0.7), (0.2, 0.7), (0.2, 0.9)]
    
    # Motor
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float("inf"))
    right_motor.setVelocity(0.0)
    
    # Sonor Sensor
    so_sen = []
    for i in range(8):
        s = f"ps{str(i)}"
        so_sen.append(robot.getDevice(s))
        so_sen[i].enable(timestep)
        dist_val.append(0)
        
        
    f_gps = robot.getDevice("front_gps")
    f_gps.enable(timestep)
    
    m_gps = robot.getDevice("mid_gps")
    m_gps.enable(timestep)
    jks = 0
    goal = goals[jks]
    
        
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        
        for i, s in enumerate(so_sen):
            dist_val[i] = s.getValue()
        
        p1 = (float(f_gps.getValues()[0]), float(f_gps.getValues()[1]))
        p2 = (float(m_gps.getValues()[0]), float(m_gps.getValues()[1]))
        
        targ_angle = calc_dir(goal, p1)
        curr_angle = calc_dir(p1, p2)
        
        if calc_dist(goal, p2) < 0.1:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            jks += 1
            print(f"{jks} goal reached {goal}")
            if jks+1 > len(goals):
                print(goal)
                print("!!! Target reached !!!")
                break;
            goal = goals[jks]
       
            
        frn_wall = max(dist_val[7], dist_val[1]) > 78
        rgt_wall = dist_val[2] > 80
        if abs(targ_angle - curr_angle) > 0.2 :
            if targ_angle > curr_angle:
                left_speed = -0.75*wanna_speed
                right_speed = wanna_speed
            else:
                right_speed = -0.75*wanna_speed
                left_speed = wanna_speed
        else: 
            left_speed = wanna_speed
            right_speed = wanna_speed
        
        
        left_motor.setVelocity(min(left_speed, 6.28))
        right_motor.setVelocity(min(right_speed, 6.28))
        
    
# create the Robot instance.
dist_val = []
max_speed = 6.28/3
wanna_speed = 6.28/3
robot = Robot()
move(robot)