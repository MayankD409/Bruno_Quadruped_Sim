# -*- coding: utf-8 -*-

from sympy import symbols, cos, sin, pi, simplify, Matrix, pprint, diff, shape
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Joint angles
theta1, theta2, theta3 = symbols('theta1 theta2 theta3')
t, omega, q = symbols('t omega q')
theta = [0.0, -0.7, 2]
theta = Matrix(theta)

################################################ D-H Parameters #####################################################################
# Front-Right
a_fr = [0.255, 0.520, 0.550]
alpha_fr = [pi/2, 0, 0]
d_fr = [0, 0, 0]
theta_fr = [theta1, theta2, theta3]

# Front-Left
a_fl = [0.255, 0.520, 0.550]
alpha_fl = [pi/2, 0, 0]
d_fl = [0, 0, 0]
theta_fl = [theta1, theta2, theta3]

# Back-Right
a_br = [0.255, 0.520, 0.550]
alpha_br = [pi/2, 0, 0]
d_br = [0, 0, 0]
theta_br = [theta1, theta2, theta3]

# Back-Left
a_bl = [0.255, 0.520, 0.550]
alpha_bl = [pi/2, 0, 0]
d_bl = [0, 0, 0]
theta_bl = [theta1, theta2, theta3]

####################################### Forward Kinematics ####################################################
# Transformation matrices0
def dh_transform_matrix(theta, d, a, alpha):
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta, d, a, alpha):
  T01 = dh_transform_matrix(theta[0], d[0], a[0], alpha[0])
  T12 = dh_transform_matrix(theta[1], d[1], a[1], alpha[1])
  T23 = dh_transform_matrix(theta[2], d[2], a[2], alpha[2])

  T02 = simplify(T01 * T12)
  T03 = simplify(T02 * T23)
  T0n = simplify(T03)

  P=T0n[:3, 3]
  print("Position of leg: \n")
  pprint(P)
  z0 = T01[:3, 2]
  z1 = T02[:3, 2]
  z2 = T0n[:3, 2]
  J = Matrix([
    [P.diff(theta[0]), P.diff(theta[1]), P.diff(theta[2])],
    [z0, z1, z2]
  ])
  return T0n, J

###################################################### Forward Kinematics Calculation (STMKC) #########################################################################

T0n_fr, J_fr = forward_kinematics(theta_fr, d_fr, a_fr, alpha_fr) # Transformation matrix & Jacobian for front_right leg
T0n_fl, J_fl = forward_kinematics(theta_fl, d_fl, a_fl, alpha_fl) # Transformation matrix & Jacobian for front_left leg
T0n_br, J_br = forward_kinematics(theta_br, d_br, a_br, alpha_br) # Transformation matrix & Jacobian for back_right leg
T0n_bl, J_bl = forward_kinematics(theta_bl, d_bl, a_bl, alpha_bl) # Transformation matrix & Jacobian for back left leg

print("Transformation Matrix of Front left leg: \n")
pprint(T0n_fl)
print("Transformation Matrix of Front right leg: \n")
pprint(T0n_fr)
print("Transformation Matrix of Back left leg: \n")
pprint(T0n_bl)
print("Transformation Matrix of Back right leg: \n")
pprint(T0n_br)
print("Jacobian of Front left leg: \n")
pprint(J_fl)
print("Jacobian of Front right leg: \n")
pprint(J_fr)
print("Jacobian of Back left leg: \n")
pprint(J_bl)
print("Jacobian of Back right leg: \n")
pprint(J_br)

################################################################# Forward Kinematics Validation #########################################################

def forward_validation(T0n, a):
  T0n_vals = T0n.subs({theta1: a[0], theta2: a[1], theta3: a[2]})
  P=T0n_vals[:3, 3]
  return T0n_vals, P

a1 = [0,0,0]
a2 = [pi/2, 0, pi/2]
a3 = [pi/2, 0, 0]
a4 = [0, pi/2, pi/2]
a5 = [pi, pi/2, pi/2]
T0n_vals_fr1, P_fr1 = forward_validation(T0n_fr, a1)
T0n_vals_fr2, P_fr2 = forward_validation(T0n_fr, a2)
T0n_vals_fr3, P_fr3 = forward_validation(T0n_fr, a3)
T0n_vals_fr4, P_fr4 = forward_validation(T0n_fr, a4)
T0n_vals_fr5, P_fr5 = forward_validation(T0n_fr, a5)
print("Transformation Matrix for [pi, pi/2, pi/2]: \n")
pprint(T0n_vals_fr5)
print("foot position")
pprint(P_fr5)

############################################# Setting up HyperParameters ########################################################################

time_step = 0.0001
num_points = 500
omega = 2*pi/10
damping_factor = 0.6

"""## Back Right Leg"""

br_x=[]
br_y=[]
br_z=[]
joint_br=[]
for t in range(num_points):
  if t<=(num_points/2):
    X = 0.1*omega*cos(2*pi*t/num_points)
    Z = -0.1*omega*sin(2*pi*t/num_points)
  else:
    X = 0.0
    Z = 0.04
  Vel = Matrix([X, 0, Z, 0.0, 0.0, 0.0])
  joint_br.append([theta[0], theta[1], theta[2]])
  Jacobian_numerical = J_br.subs({theta1: theta[0], theta2: theta[1],
                                  theta3: theta[2]})
  Jacobian_numerical = simplify(Jacobian_numerical)

  # Damped least squares
  Jac_damped = Jacobian_numerical.T * (Jacobian_numerical * Jacobian_numerical.T + damping_factor**2 * Matrix.eye(6)).inv()
  theta_dot = (Jac_damped * Vel).evalf()
  theta = theta + theta_dot*time_step
  T0n_vals = T0n_br.evalf(subs={theta1: theta[0], theta2: theta[1],
                              theta3: theta[2]})
  br_x.append(T0n_vals[3])
  br_y.append(T0n_vals[7])
  br_z.append(T0n_vals[11])

"""## Front Left"""

fl_x=[]
fl_y=[]
fl_z=[]
joint_fl=[]
for t in range(num_points):
  if t<=(num_points/2):
    X = 0.1*omega*cos(2*pi*t/num_points)
    Z = -0.1*omega*sin(2*pi*t/num_points)
  else:
    X = 0.0
    Z = 0.04
  Vel = Matrix([X, 0, Z, 0.0, 0.0, 0.0])
  joint_fl.append([theta[0], theta[1], theta[2]])
  Jacobian_numerical = J_fl.subs({theta1: theta[0], theta2: theta[1],
                                  theta3: theta[2]})
  Jacobian_numerical = simplify(Jacobian_numerical)

  # Damped least squares
  Jac_damped = Jacobian_numerical.T * (Jacobian_numerical * Jacobian_numerical.T + damping_factor**2 * Matrix.eye(6)).inv()
  theta_dot = (Jac_damped * Vel).evalf()
  theta = theta + theta_dot*time_step
  T0n_vals = T0n_fl.evalf(subs={theta1: theta[0], theta2: theta[1],
                              theta3: theta[2]})
  fl_x.append(T0n_vals[3])
  fl_y.append(T0n_vals[7])
  fl_z.append(T0n_vals[11])

"""## Front Right"""

fr_x=[]
fr_y=[]
fr_z=[]
joint_fr=[]
for t in range(num_points):
    X = 0.0
    Z = 0.02
    Vel = Matrix([X, 0, Z, 0.0, 0.0, 0.0])
    joint_fr.append([theta[0], theta[1], theta[2]])
    Jacobian_numerical = J_fr.subs({theta1: theta[0], theta2: theta[1],
                                    theta3: theta[2]})
    Jacobian_numerical = simplify(Jacobian_numerical)

    # Damped least squares
    Jac_damped = Jacobian_numerical.T * (Jacobian_numerical * Jacobian_numerical.T + damping_factor**2 * Matrix.eye(6)).inv()
    theta_dot = (Jac_damped * Vel).evalf()
    theta = theta + theta_dot*time_step
    T0n_vals = T0n_fr.evalf(subs={theta1: theta[0], theta2: theta[1],
                               theta3: theta[2]})
    fr_x.append(T0n_vals[3])
    fr_y.append(T0n_vals[7])
    fr_z.append(T0n_vals[11])

"""## Back Left"""

bl_x=[]
bl_y=[]
bl_z=[]
joint_bl=[]
for t in range(num_points):
    X = 0.0
    Z = 0.02
    Vel = Matrix([X, 0, Z, 0.0, 0.0, 0.0])
    joint_bl.append([theta[0], theta[1], theta[2]])
    Jacobian_numerical = J_bl.subs({theta1: theta[0], theta2: theta[1],
                                    theta3: theta[2]})
    Jacobian_numerical = simplify(Jacobian_numerical)

    # Damped least squares
    Jac_damped = Jacobian_numerical.T * (Jacobian_numerical * Jacobian_numerical.T + damping_factor**2 * Matrix.eye(6)).inv()
    theta_dot = (Jac_damped * Vel).evalf()
    theta = theta + theta_dot*time_step
    T0n_vals = T0n_bl.evalf(subs={theta1: theta[0], theta2: theta[1],
                               theta3: theta[2]})
    bl_x.append(T0n_vals[3])
    bl_y.append(T0n_vals[7])
    bl_z.append(T0n_vals[11])

# Create a grid of subplots with 2 rows and 2 columns
fig, axs = plt.subplots(2, 2)

# Plot 1: Top left subplot
axs[1, 1].plot(br_z, br_x)
axs[1, 1].set_xlabel("z")
axs[1, 1].set_ylabel("x")
axs[1, 1].set_title("Back Right Leg")
axs[1, 1].axis("equal")
axs[1, 1].grid(False)

# Plot 2: Top right subplot
axs[1, 0].plot(bl_z, bl_x)  # Replace with your desired plot
axs[1, 0].set_xlabel("z")
axs[1, 0].set_ylabel("x")
axs[1, 0].set_title("Back Left Leg")
axs[1, 0].axis("equal")
axs[1, 0].grid(False)
# Add other formatting and plotting commands as needed

# Plot 3: Bottom left subplot
axs[0, 1].plot(fr_z, fr_x)  # Replace with your desired plot
axs[0, 1].set_xlabel("z")
axs[0, 1].set_ylabel("x")
axs[0, 1].set_title("Front Right Leg")
axs[0, 1].axis("equal")
axs[0, 1].grid(False)
# Add other formatting and plotting commands as needed

# Plot 4: Bottom right subplot
axs[0, 0].plot(fl_z, fl_x)  # Replace with your desired plot
axs[0, 0].set_xlabel("z")
axs[0, 0].set_ylabel("x")
axs[0, 0].set_title("Front Left Leg")
axs[0, 0].axis("equal")
axs[0, 0].grid(False)
# Add other formatting and plotting commands as needed

# Adjust the layout to avoid overlapping titles and labels
fig.tight_layout()

# Show the plot
plt.show()
