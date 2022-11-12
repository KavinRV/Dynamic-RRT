"""bug_0_algo controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from sympy import Polygon, Point, Ellipse
import numpy as np


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
    v1, v2, v3, v4 = map(Point, poly)
    poly = Polygon(v1, v2, v3, v4)
    pnt = p

    return poly.distance(Point(pnt[0], pnt[1])) * (int(poly.encloses_point(Point(pnt[0], pnt[1]))) * (-2) + 1)


def calc_poly_tang(poly, p):
    """
    :param p: 0-->p1, 1-->p2
    :param poly: Polygon represented by the list of tuples(x, y) of vertices.
    :return: Returns numpy vector pointing the polygon from the point.
    """
    v1, v2, v3, v4 = map(Point, poly)
    poly = Polygon(v1, v2, v3, v4)
    pnt = p

    rad = poly.distance(Point(pnt))
    circ = Ellipse(Point(pnt), rad, rad)
    cls_pnt = poly.intersection(circ)
    return np.array(cls_pnt[0]) - np.array(pnt)


def poly_intersection(poly1, poly2):
    """
    Finds intersections b/w two pol
    :param poly1: Polygon represented by the list of tuples(x, y) of vertices.
    :param poly2: Polygon represented by the list of tuples(x, y) of vertices.
    :return: List of points of intersections b/w poly1 and poly2
    """
    v11, v12, v13, v14 = map(Point, poly1)
    v21, v22, v23, v24 = map(Point, poly2)

    poly1 = Polygon(v11, v12, v13, v14)
    poly2 = Polygon(v21, v22, v23, v24)

    ret = list(map(list, poly1.intersection(poly2)))
    return ret


def move(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    # Motor
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float("inf"))
    right_motor.setVelocity(0.0)
    
    # Proximity Sensor
    p_sen = []
    for i in range(8):
        s = f"ps{str(i)}"
        p_sen.append(robot.getDistanceSensor(s))
        p_sen[i].enable(timestep)
        
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        left_wall = p_sen[5].getValue() > 80
        right_wall = p_sen[2].getValue() > 80
        front_wall = p_sen[7].getValue() > 80
    
        # Process sensor data here.
        
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        left_motor.setVelocity(6.0)
        right_motor.setVelocity(6.0)
        
    
# create the Robot instance.
robot = Robot()
move(robot)


# Enter here exit cleanup code.
