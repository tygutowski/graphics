import numpy as np
from vedo import *


def forwardKinematics(theta_arr):

    #
    # Construction of transformation matrices
    # A translation matrix (on the right) composed of the arm length, multiplied by the rotation matrix
    #

    base_x = 5
    base_y = 5
    length_arr = [2, 5, 5, 5]

    T01 = np.array([[np.cos(theta_arr[0]), -np.sin(theta_arr[0]), 0.0, 5],
                    [np.sin(theta_arr[0]), np.cos(theta_arr[0]) , 0.0, 5],
                    [0.0                 , 0.0                  , 1.0, 0.0],
                    [0.0                 , 0.0                  , 0.0, 1.0]]
                   )
    T12 = np.array([[np.cos(theta_arr[1]) , 0.0, np.sin(theta_arr[1]), 0.0],
                    [0.0                  , 1.0, 0.0                 , 0.0],
                    [-np.sin(theta_arr[1]), 0.0, np.cos(theta_arr[1]), length_arr[0]],
                    [0.0                  , 0.0, 0.0                 , 1.0]]
                   )
    T23 = np.array([[np.cos(theta_arr[2]) , 0.0, np.sin(theta_arr[2]), 0.0],
                    [0.0                  , 1.0, 0.0                 , 0.0],
                    [-np.sin(theta_arr[2]), 0.0, np.cos(theta_arr[2]), length_arr[1]],
                    [0.0                  , 0.0, 0.0                 , 1.0]]
                   )
    T34 = np.array([[np.cos(theta_arr[3]) , 0.0, np.sin(theta_arr[3]), 0.0],
                    [0.0                  , 1.0, 0.0                 , 0.0],
                    [-np.sin(theta_arr[3]), 0.0, np.cos(theta_arr[3]), length_arr[2]],
                    [0.0                  , 0.0, 0.0                 , 1.0]]
                   )

    # Calculate all transforms to global frame
    T02 = T01 @ T12
    T03 = T01 @ T12 @ T23
    T04 = T01 @ T12 @ T23 @ T34

    return [T01, T02, T03, T04]

def animate_robot(angles):
    while(True):
        print(angles[2])
        angles = np.array([angles[0], angles[1], angles[2], angles[3]+.07])
        drawRobotArm(angles)
    

def drawRobotArm(phiArray):
    global frame, base, component1, component2, component3, angles
    transform_arr = forwardKinematics(phiArray)  # Array of all local-to-global transforms
    
    base.apply_transform(transform_arr[0])          # Applying transforms
    component1.apply_transform(transform_arr[1])
    component2.apply_transform(transform_arr[2])
    component3.apply_transform(transform_arr[3])
    plt = show([base, component1, component2, component3], __doc__, bg='black', bg2='bb', interactive=True, axes=1,
               viewup='z')


component3 = Sphere(r=0.7, pos=(0, 0, 0)) + Cylinder(r=0.5, height=5, pos=(0, 0, 2.5)) + Sphere(r=0.7, pos=(0, 0, 5), c="y")
component2 = Sphere(r=0.7, pos=(0, 0, 0)) + Cylinder(r=0.5, height=5, pos=(0, 0, 2.5))
component1 = Sphere(r=0.7, pos=(0, 0, 0)) + Cylinder(r=0.5, height=5, pos=(0, 0, 2.5))
base = Cube(pos=(0, 0, 0,), side=3)

# starting angles
angles = np.array([[0, 0, 0], # angle 01 [x y z]
                   [0, 0, 0], # angle 12
                   [0, 0, 0]]) # angle 23

# Set up initial frame
transform_arr = forwardKinematics(angles)       # Applying transforms
base.apply_transform(transform_arr[0])
component1.apply_transform(transform_arr[1])
component2.apply_transform(transform_arr[2])
component3.apply_transform(transform_arr[3])

# anim_video = Video("armanim.mp4", duration=5, fps=24, backend="ffmpeg")

# Build the graphical scene with all objects and axes
plt = Plotter(size=(1050, 600))
plt += [base, component1, component2, component3, __doc__]
plt.background("black", "w").add_global_axes(axtype=1).look_at(plane='yz')

plt.show()

animate_robot(angles)