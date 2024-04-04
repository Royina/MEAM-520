    # def DH_transformation (DH_params, q, joint_number):
def DH_transformation(DH_params, q, joint_number):
    """
    INPUT: q            -   modified vector of joint angles [0, q0, q1, q2, q3, q4, q5, q6] - 0 added in the beginning by <q = np.insert(q, 0, 0)> to reshape it to 8x1 to enable for loop
           DH_params    -   vector of DH_param constants [theta_offset, d, a, alpha]
           joint_number -   which joint
    In DH Parameters, theta = theta_offset + joint variable - for revolute
                For usage here we have used only the offsets for theta (for neutral position).
                Joint variables will be passed additionally to the function and added to offsets to formulate the correct matrices
    """

    DH = DH_params[joint_number]

    theta = DH[0] + q[joint_number]          #theta = theta_offset + joint variable
    d = DH[1]
    a = DH[2]
    alpha = DH[3]

    # print('\n',DH, '\n')
    A_matrix = np.zeros((4,4))

    A_matrix[0][0] = cos(theta)
    A_matrix[0][1] = -1 * sin(theta) * cos(alpha)
    A_matrix[0][2] = sin(theta) * sin(alpha)
    A_matrix[0][3] = a * cos(theta)
    A_matrix[1][0] = sin(theta)
    A_matrix[1][1] = cos(theta) * cos(alpha)
    A_matrix[1][2] = -1 * cos(theta) * sin(alpha)
    A_matrix[1][3] = a * sin(theta)
    A_matrix[2][0] = 0
    A_matrix[2][1] = sin(alpha)
    A_matrix[2][2] = cos(alpha)
    A_matrix[2][3] = d
    A_matrix[3][0] = 0
    A_matrix[3][1] = 0
    A_matrix[3][2] = 0
    A_matrix[3][3] = 1

    return A_matrix





import numpy as np
from math import cos, sin, pi
"""
INPUT: q            -   vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
       DH_params    -   vector of DH_param constants [theta_offset, d, a, alpha]
       joint_number -   which joint
In DH Parameters, theta = theta_offset + joint variable - for revolute
            For usage here we have used only the offsets for theta (for neutral position).
            Joint variables will be passed additionally to the function and added to offsets to formulate the correct matrices
"""
# joint_number = 4
q = np.array((7,1))
q = [0,0,0,-pi/2,0,pi/2,pi/4]
q_modified = np.insert(q, 0 ,0)
# print(q , '\n')
q_correction=np.array((7,1))
q_correction = [0,0,0,0,-pi/2,0,pi/2,pi/4]
q_modified = np.subtract(q_modified, q_correction)

DH_params  =   [[0,     0.141,        0,         0],
                [0,     0.192,        0,      pi/2],
                [0,     0.000,        0,     -pi/2],
                [0,     0.195+0.121,  0.0825, pi/2],
                [pi/2,  0.000,        0.0825, pi/2],
                [0,     0.125+0.259,  0,     -pi/2],
                [-pi/2, 0.000,        0.088,  pi/2],
                [0,     0.051+0.159,  0,         0]]


A = np.zeros((8,4,4))
for i in range(0,8):
    A[i] = DH_transformation(DH_params, q_modified, i)
    # print(i, "\n \n", A[i], "\n")

T_wrt_0 = np.zeros((7,4,4))
T_wrt_0[0] = np.dot(A[0], A[1])

for j in range(1,7):
    T_wrt_0[j] = np.dot(T_wrt_0[j-1], A[j+1])

print("Final T \n \n", T_wrt_0[6],"\n")


joint_position = np.array((8,3))

# print(T_wrt_0[1,:-1,:-1])

jointPositions = np.zeros((8,3))

jointPositions[0] = A[0,:-1,3]
jointPositions[1] = T_wrt_0[0,:-1,3]
jointPositions[2] = T_wrt_0[1,:-1,3] + 0.195*(np.dot(T_wrt_0[1,:-1,:-1], np.array([0,0,1])))
jointPositions[3] = T_wrt_0[2,:-1,3]
jointPositions[4] = T_wrt_0[3,:-1,3] + 0.125*(np.dot(T_wrt_0[3,:-1,:-1], np.array([0,0,1])))
jointPositions[5] = T_wrt_0[4,:-1,3] + 0.015*(np.dot(T_wrt_0[4,:-1,:-1], np.array([0,0,-1])))
jointPositions[6] = T_wrt_0[5,:-1,3] + 0.051*(np.dot(T_wrt_0[5,:-1,:-1], np.array([0,0,1])))
jointPositions[7] = T_wrt_0[6,:-1,3]


print(jointPositions)
