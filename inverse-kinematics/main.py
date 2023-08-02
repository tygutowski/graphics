# multimodal function
import numpy as np
import matplotlib.pyplot as plt
from vedo import *

def goal_attraction(e_Phi: np.ndarray, g: np.ndarray) -> float:
    """Computes the value of the goal-attraction cost 
    
    Args:
        e_Phi (np.ndarray): A 3x1 column matrix containing the 
                        (ex,ey,ez) coordinates of the current 
                        location of the end effector.
                        
        g (np.ndarray): A 3x1 column matrix containing the 
                        (gx,gy,gz) coordinates of the 
                        location of the goal point.

    Returns:
        cost (float):   The calculated cost 
    
      """
    
    c = np.linalg.norm(e_Phi - g)
    return c    # float 

def F_R(d: float) -> float:
    """Computes the value of the field potential for 
       obstacle-avoidance cost 
    
    Args:
        d (np.float): Distance between the end-effector and 
                      the obstacle.                        

    Returns:
        cost (float):   The calculated cost 
    
      """
    
    # Assume that all objects have the same size. 
    # Radius (approx) of the object. It might 
    # need to be a bit larger than the actual 
    # radius of the object.
    R = 3
    cost = 2;
    if (d <= R) and (d > 0):
        cost = np.log(R/d)
    elif (d > R):
        cost = 0  
    
    return cost   # float 
    
def L(phi:float, min_:int, max_:int, delta_:int) -> float:
    # Assume all limits and delta to be the same
    phi_min = min_        # Joint's minimum angle limit.                        
    
    phi_max = max_       # Joint's maximum angle limit.
    
    delta   = delta_       # The angular distance from each of 
                       # the limits after which the limit 
                       # function vanishes.
    
    
    # Temp value to return 
    cost = 10
    
    # Cost function
    if (phi > phi_min) and (phi <= (phi_min+delta)):
        cost = np.log(phi / (phi-phi_min))
    elif (phi > phi_min + delta) and (phi < phi_max - delta):
        cost = 0
    elif (phi >= phi_max - delta) and (phi < phi_max):
        cost = np.log(phi / (phi_max - phi))
    
    return cost   # float 
    
def get_end_effector(Phi):
    theta_arr = np.squeeze(Phi)

    # Arm base x,y,z coordinates / translations
    arm_x = 0
    arm_y = 0
    arm_z = 2
    
    # Length of each arm component extending from join
    length_arr = [2, 5, 5, 5] 
    
    # Local transformation matrices 
    
    # Frame 1 written w.r.t. Frame 1 (global)
    T_01 = np.array([[np.cos(theta_arr[0]), -np.sin(theta_arr[0]), 0.0, arm_x],
                    [np.sin(theta_arr[0]), np.cos(theta_arr[0]) , 0.0, arm_y],
                    [0.0                 , 0.0                  , 1.0, arm_z],
                    [0.0                 , 0.0                  , 0.0, 1.0]]
                   )
    # Frame 1 written w.r.t. Frame 2
    T_12 = np.array([[np.cos(theta_arr[1]) , 0.0, np.sin(theta_arr[1]), 0.0],
                    [0.0                  , 1.0, 0.0                 , 0.0],
                    [-np.sin(theta_arr[1]), 0.0, np.cos(theta_arr[1]), length_arr[0]],
                    [0.0                  , 0.0, 0.0                 , 1.0]]
                   )
    # Frame 3 written w.r.t. Frame 2
    T_23 = np.array([[np.cos(theta_arr[2]) , 0.0, np.sin(theta_arr[2]), 0.0],
                    [0.0                  , 1.0, 0.0                 , 0.0],
                    [-np.sin(theta_arr[2]), 0.0, np.cos(theta_arr[2]), length_arr[1]],
                    [0.0                  , 0.0, 0.0                 , 1.0]]
                   )
    # Frame 4 written w.r.t. Frame 3
    T_34 = np.array([[np.cos(theta_arr[3]) , 0.0, np.sin(theta_arr[3]), 0.0],
                    [0.0                  , 1.0, 0.0                 , 0.0],
                    [-np.sin(theta_arr[3]), 0.0, np.cos(theta_arr[3]), length_arr[2]],
                    [0.0                  , 0.0, 0.0                 , 1.0]]
                   ) 
    
    # Local-to-global transformation matrices 
    T_04 = T_01 @ T_12 @ T_23 @ T_34                    # Frame 4 written w.r.t. Frame 0
    e = T_04[0:3, 3]
    
    return e   
# Cost function 
def C(Phi, goal_location, obstacle_locations, angle_min, angle_max, angle_delta): 
    
    # Goal (target) location 
    g = goal_location

    # End-effector location 
    e_Phi = get_end_effector(Phi)

    # Location of obstacles 
    obstacle = obstacle_locations # A 2d array
    num_obstacles = obstacle.size # Number of rows of 2d array

    cost = 0

    # Goal attraction
    cost = np.linalg.norm(e_Phi - g) 

    # Obstacle avoidance penalty
    cost += sum(F_R(np.linalg.norm(e_Phi - obstacle[:j])) for j in range(num_obstacles))

    # Joint-range limit
    Phi = np.squeeze(Phi)
    cost += sum(L(phi, angle_min, angle_max, angle_delta) for phi in Phi)  
    return cost

def J(end_effector_function, Phi):

    e = end_effector_function
    
    # Delta change in each angle
    deltas = [0.1, 0.1, 0.1, 0.1]
    
    # Delta change in each angle in matrix form, for easier addition
    deltas_matrix = np.array([
                      [0.1, 0, 0, 0],
                      [0, 0.1, 0, 0],
                      [0, 0, 0.1, 0],
                      [0, 0, 0, 0.1]])
    
    # Initilize Jacobian matrix
    J = np.empty((3, 1))
    
    # For each angle
    for i in range(4): 
        
        # Calculate end-effector delta
        e_delta = ( e(Phi + deltas_matrix[i]) - e(Phi))
        
        # Calculate numerical estimation of partial derivative,
        DxDphi = e_delta[0] / deltas[i]
        DyDphi = e_delta[1] / delta[i]
        DzDphi = e_delta[2] / delta[i]
        
        # Append resulting values to jacobian matrix
        J = np.concatenate( (J, np.array([[DxDphi], [DyDphi], [DzDphi]])), axis=1)
    
    """Computes the value of the function at x
    
    Args:
        end_effector_function: handle to end_effector function 
 
    Returns:
        Jacobian (np.ndarray): A 3x4 Jacobian matrix
    """

    # Filter out extra columns/rows and return
    return J[:1:5]

# Cost minimization
# Seems similar to calculating Jacobian
def calculate_gradient(PhiArr, goal, obstacles, ang_max, ang_min, lim_delta):
    
    # Delta change in each angle/phi
    deltas = [0.1, 0.1, 0.1, 0.1]
    
    # Delta change in each angle in matrix form, for easier addition
    deltas_matrix = np.array([[0.1, 0, 0, 0],
                              [0, 0.1, 0, 0],
                              [0, 0, 0.1, 0],
                              [0, 0, 0, 0.1]])
    
    # Empty gradient
    gradient = np.empty((1, 1))
    
    # For each angle in arm
    for i in range(4) :
        
        # C(Φ + ΔΦₙ) 
        cost_with_PhiDelta = C(PhiArr + deltas_matrix[i], goal, obstacles, ang_max, ang_min, lim_delta)
        # C(Φ)
        cost_without_PhiDelta = C(PhiArr,  goal, obstacles, ang_max, ang_min, lim_delta)
        
        # Calculate components of gradient
        # ( C(Φ + ΔΦₙ) -  C(Φ) )/ ΔΦₙ
        deltaC = (cost_with_PhiDelta - cost_without_PhiDelta) / deltas[i]
        
        # Add component to gradient matrix
        np.append(gradient, deltaC)

    return gradient


def animate_robot(p, e, goal):
    
    # Obstacle locations. Change these to whatever you want
    obstacles = np.array([[-5,0,-5],
                          [0,5,5],
                          [0,0,0]])
    # Angle limits. Change these to whatever you want
    # Set to full range of motion by default
    ang_max = 3.141 # 180 deg
    ang_min = 0.785 # 45 deg
    ang_delta = 0.174 # 10 deg
    
    # Steps 1, 2, 3, 4, and 5 respectively
    PhiArr = p.transpose()
    end_effector = e.transpose()
    goal_pos = goal.transpose()
    distance = 1
    gradient_step = 0.01
    
    while (goal_attraction(end_effector, goal_pos) > distance):
    
        # Calculate new joint-angle (step 7)
        # Also just replace the old config with the new one (step 10)
        PhiArr = PhiArr - gradient_step * calculate_gradient(PhiArr, goal_pos, obstacles, ang_max, ang_min, ang_delta)
        
        # Draw robot arm (step 8)
        drawRobotArm(PhiArr)
        
        # Calculate new end-effector location, update (step 9)
        end_effector = get_end_effector(PhiArr)
        print("END EFFECTOR POSITION",end="")
        print(end_effector)
def forwardKinematics(theta_arr):
    print("FORWARD",end="")
    print(theta_arr)
    #
    # Construction of transformation matrices
    # A translation matrix (on the right) composed of the arm length, multiplied by the rotation matrix
    #

    base_x = 5
    base_y = 5
    length_arr = [2, 5, 5, 5]

    T01 = np.array([[np.cos(theta_arr[0]), -np.sin(theta_arr[0]), 0.0, 0],
                    [np.sin(theta_arr[0]), np.cos(theta_arr[0]) , 0.0, 0],
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

def drawRobotArm(phiArray):
    phiArray = np.squeeze(phiArray)
    phiArray[0] = phiArray[0]%6.28319
    phiArray[1] = phiArray[1]%6.28319
    phiArray[2] = phiArray[2]%6.28319
    phiArray[3] = phiArray[3]%6.28319
    transform_arr = forwardKinematics(phiArray)       # Applying transforms
    base.apply_transform(transform_arr[0])
    component1.apply_transform(transform_arr[1])
    component2.apply_transform(transform_arr[2])
    component3.apply_transform(transform_arr[3])
    plt = show([base, component1, component2, component3], __doc__, bg='black', bg2='bb', interactive=True, axes=1,
               viewup='z')
def main(angles):
    starting_angles = angles
    goal_position = np.array([0, 0, 10])
    end_effector_starting_position = get_end_effector(starting_angles)
    animate_robot(starting_angles, end_effector_starting_position, goal_position)


component3 = Sphere(r=0.3, pos=(0, 0, 0)) + Cylinder(r=0.2, height=5, pos=(0, 0, 2.5)) + Sphere(r=0.3, pos=(0, 0, 5), c="y")
component2 = Sphere(r=0.3, pos=(0, 0, 0)) + Cylinder(r=0.25, height=5, pos=(0, 0, 2.5))
component1 = Sphere(r=0.3, pos=(0, 0, 0)) + Cylinder(r=0.3, height=5, pos=(0, 0, 2.5))
base = Cube(pos=(0, 0, 0), side=3)


angles = np.array([0, 1.5708, 4.71239, 0]) # 90 and 270 deg
transform_arr = forwardKinematics(angles)
base.apply_transform(transform_arr[0])
component1.apply_transform(transform_arr[1])
component2.apply_transform(transform_arr[2])
component3.apply_transform(transform_arr[3])

plt = Plotter(size=(1050, 600))
plt += [base, component1, component2, component3, __doc__]
plt.background("black", "w").add_global_axes(axtype=1).look_at(plane='yz')
plt.show()

main(angles)
while(True):
    plt.show()