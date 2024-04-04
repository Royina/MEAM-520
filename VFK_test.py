def calculate_transformation_matrix(q, joint_number):
    # Ensure the input dimensions are correct

    # Define symbolic vectors for DH parameters
    alpha = sp.Array([0.141, 0.192, 0.000, 0.195+0.121, 0.000, 0.125+0.259, 0.000, 0.051+0.159])
    a = sp.Array([0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0])
    d = sp.Array([0, -pi/2, pi/2, pi/2, pi/2, -pi/2, pi/2, 0])
    theta_offset = sp.Array([0, 0, 0, 0, pi/2, 0, -pi/2, 0])
    theta = sp.zeros(8,1)

    for i in range(0,8):
        theta[i] = q[i] + theta_offset[i]
    # print(theta[i],'\n')
    # print(alpha, '\n')
    # print(a, "\n")
    # print(d, "\n")
    print(theta)


    # Define the transformation matrix elements using DH parameters
    A_matrix = sp.zeros(4,4)
    A_matrix[0,0] = A_matrix[0,0] + sp.cos(theta[joint_number])
    A_matrix[0,1] = -1 * sp.sin(theta[joint_number]) * sp.cos(alpha[joint_number])
    A_matrix[0,2] = sp.sin(theta[joint_number]) * sp.sin(alpha[joint_number])
    A_matrix[0,3] = a*sp.cos(theta[joint_number])
    A_matrix[1,0] = sp.sin(theta[joint_number])
    A_matrix[1,1] = sp.cos(theta[joint_number]) * sp.cos(alpha[joint_number])
    A_matrix[1,2] = -1 * sp.cos(theta[joint_number]) * sp.sin(alpha[joint_number])
    A_matrix[1,3] = a * sp.sin(theta[joint_number])
    A_matrix[2,0] = 0
    A_matrix[2,1] = sp.sin(alpha[joint_number])
    A_matrix[2,2] = sp.cos(alpha[joint_number])
    A_matrix[2,3] = d
    A_matrix[3,0] = 0
    A_matrix[3,1] = 0
    A_matrix[3,2] = 0
    A_matrix[3,3] = 1

    # for i in range(0,4):
    #     for j in range(0,4):
            # print(A_matrix[i,j])
    return A_matrix

import sympy as sp
from math import pi

q = sp.symarray('q', 8)
q0 = 0

# print(q)
calculate_transformation_matrix(q, 0)
