# Brendan Neal and Adam Lobo
# ENPM662 Project 2 Inverse Kinematics Validation


## ------------------------Importing Libraries-------------------------##
from sympy import *
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

init_printing(use_unicode=False, wrap_line=False)

## ------------------------Variable Definitions------------------------##

# D-H Parameters
alpha, a, d, th = symbols('alpha a d theta')

# Symbolic Joint Variables for UR10
th1, th2, th3, th4 = symbols('th1 th2 th3 th4')


# Defining Parameters as found in Report in Arrays
alpha_array = [pi/2, 0, pi/2, 0]
theta_array = [th1, th2, (pi/2)+th3, (pi/2)+th4]
d_array = [26*25.4, 0, 0, 23*25.4]  # mm
a_array = [0, 21*25.4, 0, 0]  # mm

## ------------Transformation Matrices Definitions--------------------##

Rz = Matrix([[cos(th), -sin(th), 0, 0],
             [sin(th),  cos(th), 0, 0],
             [0,        0, 1, 0],
             [0,        0, 0, 1]])

Tz = Matrix([[1,  0,  0,  0],
             [0,  1,  0,  0],
             [0,  0,  1,  d],
             [0,  0,  0,  1]])

Tx = Matrix([[1,  0,  0,  a],
             [0,  1,  0,  0],
             [0,  0,  1,  0],
             [0,  0,  0,  1]])

Rx = Matrix([[1,        0,        0,  0],
             [0,  cos(alpha), -sin(alpha),  0],
             [0,  sin(alpha),  cos(alpha),  0],
             [0,        0,        0,  1]])

## ------------Computing Each Row's Transformation Matrix--------------##
T1 = Rz*Tz*Tx*Rx
T2 = Rz*Tz*Tx*Rx
T3 = Rz*Tz*Tx*Rx
T4 = Rz*Tz*Tx*Rx

# Substituting Params into Each Transformation Matrix
T1 = T1.subs(alpha, alpha_array[0]).subs(
    th, theta_array[0]).subs(d, d_array[0]).subs(a, a_array[0])
T2 = T2.subs(alpha, alpha_array[1]).subs(
    th, theta_array[1]).subs(d, d_array[1]).subs(a, a_array[1])
T3 = T3.subs(alpha, alpha_array[2]).subs(
    th, theta_array[2]).subs(d, d_array[2]).subs(a, a_array[2])
T4 = T4.subs(alpha, alpha_array[3]).subs(
    th, theta_array[3]).subs(d, d_array[3]).subs(a, a_array[3])

## ----------Defining Transformation Matrices wrt Zero Frame-----------##
H0_1 = T1
H0_2 = T1*T2
H0_3 = T1*T2*T3
H0_4 = T1*T2*T3*T4

print("Final Transformation Matrix:")
pprint(H0_4)

## ----------------------Extracting P from H0_6------------------------##
P = Matrix([[H0_4[3]], [H0_4[7]], [H0_4[11]]])

## -----------Computing General Form of Partial Derivatives------------##

Par1 = diff(P, th1)
Par2 = diff(P, th2)
Par3 = diff(P, th3)
Par4 = diff(P, th4)


## -----------------Extracting Z from each H Matrix--------------------##
Z0_1 = Matrix([[H0_1[2]], [H0_1[6]], [H0_1[10]]])
Z0_2 = Matrix([[H0_2[2]], [H0_2[6]], [H0_2[10]]])
Z0_3 = Matrix([[H0_3[2]], [H0_3[6]], [H0_3[10]]])
Z0_4 = Matrix([[H0_4[2]], [H0_4[6]], [H0_4[10]]])

## -------------------Printing Jacobian Components---------------------##
print('Partial Derivative of P wrt q1:')
pprint(Par1)

print('Partial Derivative of P wrt q2')
pprint(Par2)

print('Partial Derivative of P wrt q3:')
pprint(Par3)

print('Partial Derivative of P wrt q4:')
pprint(Par4)

print('Z0_1:')
pprint(Z0_1)

print('Z0_2:')
pprint(Z0_2)

print('Z0_3:')
pprint(Z0_3)

print('Z0_4:')
pprint(Z0_4)


## -------------------Forming Jacobian Matrix---------------------##

# Components
J1 = Matrix([[Par1], [Z0_1]])
J2 = Matrix([[Par2], [Z0_2]])
J3 = Matrix([[Par3], [Z0_3]])
J4 = Matrix([[Par4], [Z0_4]])


# Full Jacobian
J = Matrix([[J1, J2, J3, J4]])
print("J :")
pprint(J)


## -------------------Planning Trajectory 1---------------------##
'''Picking Up the Cup'''
# Plotting variables
X = []
Y = []
Z = []
# Defining my initial joint angles.
q1 = 0.0002
q2 = 0.0001
q3 = -0.0001
q4 = 0.0001

q1_array_step1 = []
q2_array_step1 = []
q3_array_step1 = []
q4_array_step1 = []


time = np.linspace(0, 10, num=500)
dt = 10 / 500
for t in time:
    # Define components of X_Dot
    Vx = 0.0
    Vy = 0.0
    Vz = 60
    omega_x = 0.0
    omega_y = 0.0
    omega_z = 0.0

    # Formulate X_Dot
    X_dot = Matrix([[Vx], [Vy], [Vz], [omega_x], [omega_y], [omega_z]])

    # Substitude Joint Angles Jacobian Matrix
    J_Applied = J.subs({th1: q1, th2: q2, th3: q3, th4: q4})

    # Calculate q_dot
    q_dot = (J_Applied.pinv() * X_dot).evalf()

    # Extracting Elements of q_dot
    q1_dot = q_dot[0]
    q2_dot = q_dot[1]
    q3_dot = q_dot[2]
    q4_dot = q_dot[3]

    # Numerical Integration of Each Joint Angle
    q1 = q1 + q1_dot * dt
    q2 = q2 + q2_dot * dt
    q3 = q3 + q3_dot * dt
    q4 = q4 + q4_dot * dt

    q1_array_step1.append(q1)
    q2_array_step1.append(q2)
    q3_array_step1.append(q3)
    q4_array_step1.append(q4)

    # Substitude Joint Angles into Final Transformation Matrix
    H0_4_Applied = H0_4.subs({th1: q1, th2: q2, th3: q3, th4: q4})

    # #Saving For Plotting
    X.append(H0_4_Applied[3])
    Y.append(H0_4_Applied[7])
    Z.append(H0_4_Applied[11])



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ylim(-1, 1)

ax.scatter(X, Y, Z, marker='o')

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Trajectory of End-Effector')

plt.show()

## -------------------Planning Trajectory 2---------------------##
'''Lifting Cup Up by Reversing the Order of Commands'''
q1_array_step2 = q1_array_step1[::-1]
q2_array_step2 = q2_array_step1[::-1]
q3_array_step2 = q3_array_step1[::-1]
q4_array_step2 = q4_array_step1[::-1]

## -------------------Planning Trajectory 3---------------------##
'''Placing into Basket'''
r = 44*25.1  # mm
# Defining my initial joint angles.
q1 = 0.0002
q2 = 0.0001
q3 = -0.0001
q4 = 0.0001

q1_array_step3 = []
q2_array_step3 = []
q3_array_step3 = []
q4_array_step3 = []

# Plotting variables
X = []
Y = []
Z = []


time = np.linspace(0, 30, num=200)
dt = 30 / 200
for t in time:
    # Define components of X_Dot
    Vy = ((2*pi*r)/60)*cos((2*pi*t)/60)
    Vx = -((2*pi*r)/60)*sin((2*pi*t)/60)
    Vz = 0.0
    omega_x = 0.0
    omega_y = 0.0
    omega_z = 0.0

    # Formulate X_Dot
    X_dot = Matrix([[Vx], [Vy], [Vz], [omega_x], [omega_y], [omega_z]])

    # Substitude Joint Angles Jacobian Matrix
    J_Applied = J.subs({th1: q1, th2: q2, th3: q3, th4: q4})

    # Calculate q_dot
    q_dot = (J_Applied.pinv() * X_dot).evalf()

    # Extracting Elements of q_dot
    q1_dot = q_dot[0]
    q2_dot = q_dot[1]
    q3_dot = q_dot[2]
    q4_dot = q_dot[3]

    # Numerical Integration of Each Joint Angle
    q1 = q1 + q1_dot * dt
    q2 = q2 + q2_dot * dt
    q3 = q3 + q3_dot * dt
    q4 = q4 + q4_dot * dt

    q1_array_step3.append(q1)
    q2_array_step3.append(q2)
    q3_array_step3.append(q3)
    q4_array_step3.append(q4)

    # Substitude Joint Angles into Final Transformation Matrix
    H0_4_Applied = H0_4.subs({th1: q1, th2: q2, th3: q3, th4: q4})

    # #Saving For Plotting
    X.append(H0_4_Applied[3])
    Y.append(H0_4_Applied[7])
    Z.append(H0_4_Applied[11])


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(X, Y, Z, marker='o')

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Trajectory of End-Effector')

plt.show()


## -------------------Planning Trajectory 3---------------------##
'''Placing into Basket'''
# Defining my initial joint angles.
q1 = q1_array_step3[-1]
q2 = q2_array_step3[-1]
q3 = q3_array_step3[-1]
q4 = q4_array_step3[-1]

q1_array_step4 = []
q2_array_step4 = []
q3_array_step4 = []
q4_array_step4 = []

# Plotting variables
X = []
Y = []
Z = []


time = np.linspace(0, 20, num=200)
dt = 20 / 200
for t in time:
    # Define components of X_Dot
    Vx = 15
    Vy = 0.0
    Vz = 0.0
    omega_x = 0.0
    omega_y = 0.0
    omega_z = 0.0

    # Formulate X_Dot
    X_dot = Matrix([[Vx], [Vy], [Vz], [omega_x], [omega_y], [omega_z]])

    # Substitude Joint Angles Jacobian Matrix
    J_Applied = J.subs({th1: q1, th2: q2, th3: q3, th4: q4})

    # Calculate q_dot
    q_dot = (J_Applied.pinv() * X_dot).evalf()

    # Extracting Elements of q_dot
    q1_dot = q_dot[0]
    q2_dot = q_dot[1]
    q3_dot = q_dot[2]
    q4_dot = q_dot[3]

    # Numerical Integration of Each Joint Angle
    q1 = q1 + q1_dot * dt
    q2 = q2 + q2_dot * dt
    q3 = q3 + q3_dot * dt
    q4 = q4 + q4_dot * dt
    
    q1_array_step4.append(q1)
    q2_array_step4.append(q2)
    q3_array_step4.append(q3)
    q4_array_step4.append(q4)
    
    # Substitude Joint Angles into Final Transformation Matrix
    H0_4_Applied = H0_4.subs({th1: q1, th2: q2, th3: q3, th4: q4})

    # #Saving For Plotting
    X.append(H0_4_Applied[3])
    Y.append(H0_4_Applied[7])
    Z.append(H0_4_Applied[11])


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(X, Y, Z, marker='o')

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Trajectory of End-Effector')

plt.show()