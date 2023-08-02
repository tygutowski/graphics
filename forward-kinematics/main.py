'''
Tyler Gutowski
Computer Graphics
Dr. Ribeiro
'''
from vedo import *
import numpy as np
import math

# A Cylinder class that contains a transformation matrix
class Arm:
    def __init__(self, parent=None, color="black"):
        global part_list
        self.parent = parent
        self.mesh = Cylinder([[0,0,0],[0,0,1]],.1, c=color)
        self.mesh.SetOrigin([0,0,-0.5])
        # transformation matrix
        self.transform = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        part_list.append(self)
    def set_matrix(self, a, b, c, x, y, z):
        cos = lambda theta: np.cos(theta)
        sin = lambda theta: np.sin(theta)
        self.transform = np.array([[cos(a)*cos(b),cos(a)*sin(b)*sin(c)-sin(a)*cos(c),cos(a)*sin(b)*cos(c)+sin(a)*sin(c),x],
                                   [sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c),sin(a)*sin(b)*cos(c)-cos(a)*sin(c),y],
                                   [-sin(b),cos(b)*sin(c),cos(b)*cos(c), z],
                                   [0,0,0,1]])  
        if self.parent:
            self.transform = self.transform @ self.parent.transform
        self.mesh.apply_transform(self.transform)

# Generates the base and the arms
def create_parts():
    global base
    box = Box(vector(0,0,0), .25, .25, 1, c="white`")
    base = box
    T_01 = Arm(None, "white")
    T_12 = Arm(T_01, "white")
    T_23 = Arm(T_12, "white")
    part_list[0].set_matrix(0, 0, 0, 0, 0, 0)
    part_list[1].set_matrix(0, 0, 0, 0, 0, 0)
    part_list[2].set_matrix(0, 0, 0, 0, 0, 0)

# Loops to redraw every frame
def loop_func(event):
    global part_list
    global theta
    theta += .01
    for part in part_list:
        part.set_matrix(0, theta/2, theta, 0, 0, 1)
    plt.render()

# Creates the Plotter object and adds the parts to the object
def create_plotter():
    global plt
    global base
    plt = Plotter(size=(500, 500))
    plt += [__doc__]
    for part in part_list:
        plt += part.mesh
    plt += base
    plt.background("black", "w").add_global_axes(axtype=1).look_at(plane='yz')
    return plt

# Main runner function
def main():
    create_parts()
    create_plotter()
    plt.show()
    plt.add_callback("timer", loop_func)
    plt.timer_callback("create", dt=50)
    plt.show()

base = None
plt = None
theta = 0
part_list = []
main()