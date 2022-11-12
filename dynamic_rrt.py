from tkinter import *
from random import *
import numpy as np
import time
import pickle
from PIL import ImageTk, Image


root = Tk()
root.geometry("1200x800")
canvas = Canvas(root, width=1000, height=600)

# Creating Obstacles ellipse at (180, 180), (550, 180), (380, 420), (750, 420) in tkinter with radius 40
canvas.create_oval(140, 140, 220, 220, fill="black")
canvas.create_oval(510, 140, 590, 220, fill="black")
canvas.create_oval(340, 380, 420, 460, fill="black")
canvas.create_oval(710, 380, 790, 460, fill="black")


# add boder to canvas
canvas.create_rectangle(10, 10, 1000, 600, width=2)

# add start at (20, 20) and goal at (980, 580)
canvas.create_oval(20-5, 20-5, 20+5, 20+5, fill="green")
canvas.create_oval(980-5, 580-5, 980+5, 580+5, fill="green")



class Node:
    def __init__(self, x, y, parent):
        self.x = x
        self.y = y
        self.parent = parent
        self.children = []

    def add_child(self, child):
        self.children.append(child)


root_node = Node(20, 20, None)


# function to check if a point is in the obstacle free space
def check_obstacle_free_space(x, y):
    if check_obstacle(x, y):
        return False
    else:
        return True


# function to add random points
def add_random_points():
    x = randint(30, 990)
    y = randint(30, 590)

    return x, y


# function to check if point is in obstacle
def check_obstacle(x, y):
    if (x-180)**2 + (y-180)**2 <= 50**2:
        return True
    elif (x-550)**2 + (y-180)**2 <= 50**2:
        return True
    elif (x-380)**2 + (y-420)**2 <= 50**2:
        return True
    elif (x-750)**2 + (y-420)**2 <= 50**2:
        return True
    else:
        return False


# function to find distance between two points
def find_distance(x1, y1, x2, y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)


# function to find the nearest point in the tree
def find_nearest_point(x, y, tree):
    min_dist = 100000
    nearest_point = None
    for i in range(len(tree)):
        dist = find_distance(x, y, tree[i][0], tree[i][1])
        if dist < min_dist:
            min_dist = dist
            nearest_point = i

    return nearest_point, min_dist


# function to find a point on the line between two points at a distance of r from the first point
def find_point(x1, y1, x2, y2, r):
    x = x1 + r*(x2-x1)/np.sqrt((x2-x1)**2 + (y2-y1)**2)
    y = y1 + r*(y2-y1)/np.sqrt((x2-x1)**2 + (y2-y1)**2)

    return x, y


# function to check if a point is at a distance of r from the line between two points
def check_line(x1, y1, x2, y2, x, y, r):
    if np.sqrt((x-x1)**2 + (y-y1)**2) + np.sqrt((x-x2)**2 + (y-y2)**2) - np.sqrt((x2-x1)**2 + (y2-y1)**2) <= r:
        return True
    else:
        return False


# function to check if a line between two points is in the obstacle free space
def check_line_obstacle_free_space(x1, y1, x2, y2):
    if check_line(x1, y1, x2, y2, 180, 180, 40):
        return False
    elif check_line(x1, y1, x2, y2, 550, 180, 40):
        return False
    elif check_line(x1, y1, x2, y2, 380, 420, 40):
        return False
    elif check_line(x1, y1, x2, y2, 750, 420, 40):
        return False
    else:
        return True


# function to find distance between a point and a line
def find_distance_point_line(x1, y1, x2, y2, x, y):
    return abs((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1)/np.sqrt((y2-y1)**2 + (x2-x1)**2)


# function to find a point on a line segment given speed and time
def find_point_line_segment(x1, y1, x2, y2, speed, t):
    x = x1 + speed*t*(x2-x1)/np.sqrt((x2-x1)**2 + (y2-y1)**2)
    y = y1 + speed*t*(y2-y1)/np.sqrt((x2-x1)**2 + (y2-y1)**2)

    return x, y


def RRT(m=10):
    for i in range(10000):
        x, y = add_random_points()
        if check_obstacle_free_space(x, y):
            nearest_point, min_dist = find_nearest_point(x, y, tree)
            if min_dist > m:
                x, y = find_point(tree[nearest_point][0], tree[nearest_point][1], x, y, m)
                min_dist = find_distance(tree[nearest_point][0], tree[nearest_point][1], x, y)

            px, py = find_point_line_segment(400, 120, 220, 530, 5, tree[nearest_point][3])
            px2, py2 = find_point_line_segment(770, 120, 590, 530, 5, tree[nearest_point][3])
            colide = find_distance(x, y, px, py) <= 40 or find_distance(x, y, px2, py2) <= 40

            if check_line_obstacle_free_space(tree[nearest_point][0], tree[nearest_point][1], x, y) and check_obstacle_free_space(x, y) and not colide:
                cost = tree[nearest_point][3] + min_dist/20 + 1
                tree.append([x, y, nearest_point, cost])
                canvas.create_line(tree[nearest_point][0], tree[nearest_point][1], x, y, fill="blue", width=1)
                # canvas.create_oval(x-2, y-2, x+2, y+2, fill="red")
                if find_distance(x, y, 980, 580) <= 50:
                    canvas.create_line(x, y, 980, 580, fill="blue", width=1)
                    print("Reached")
                    break

                canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill="red", tags="red")
                canvas.create_oval(px2 - 6, py2 - 6, px2 + 6, py2 + 6, fill="red", tags="red")
                canvas.pack()
                canvas.update()
                time.sleep(0.01)
                canvas.delete("red")

    path = []
    path.append([980, 580, tree[-1][3]])
    i = len(tree)-1
    while i != 0:
        path.append(tree[i][:2] + [tree[i][3]])
        i = tree[i][2]
    path.append([20, 20, 0])
    path.reverse()
    for i in range(len(path)-1):
        canvas.create_line(path[i][0], path[i][1], path[i+1][0], path[i+1][1], fill="green", width=2)

        px, py = find_point_line_segment(400, 120, 220, 530, 5, path[i][2])
        px2, py2 = find_point_line_segment(770, 120, 590, 530, 5, path[i][2])
        canvas.create_oval(px - 40, py - 40, px + 40, py + 40, fill="red", tags="red")
        canvas.create_oval(px2 - 40, py2 - 40, px2 + 40, py2 + 40, fill="red", tags="red")
        canvas.pack()
        canvas.update()
        time.sleep(0.75)
        canvas.delete("red")

    return path


tree = [[20, 20, 0, 0]]
path = RRT(m=40)

# convert the tree to a pickle file
with open('tree.pkl', 'wb') as f:
    pickle.dump(tree, f)

# convert the path to a pickle file
with open('path.pkl', 'wb') as f:
    pickle.dump(path, f)

# read file
with open('path.pkl', 'rb') as f:
    data = pickle.load(f)

# get only first two entries of each list in list of lists data
data = [x[:2] for x in data]

# divide the first coordinate by 1000 and the second by 600
data = [[x[0]/20, x[1]/20] for x in data]

print(data)

# write file
with open('dynamic_rrt_webots/controllers/path_following/new_path.pkl', 'wb') as f:
    pickle.dump(data, f)

# record canvas as a gif
canvas.postscript(file="RRT.gif", colormode='color')
print("Done")

canvas.pack()

canvas.mainloop()
