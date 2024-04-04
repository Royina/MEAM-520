import numpy as np
from math import pi, sin, cos
import sympy as sp

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        shape = (8, 4, 4)
        self.A = [[sp.Matrix.zeros(4) for _ in range(4) ] for _ in range(8)]
        self.T = [[sp.Matrix.zeros(4) for _ in range(4) ] for _ in range(8)]
        pass

    def DH_matrices(self, q):
        a = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        alpha = [0, -sp.pi/2, sp.pi/2, sp.pi/2, sp.pi/2, -sp.pi/2, sp.pi/2, 0]
        d = [0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.210]
        x = [0,0,0,0, sp.pi/2, 0, -sp.pi/2, 0]
        theta = q + x
        A = [[sp.Matrix.zeros(4) for _ in range(4) ] for _ in range(8)]

        for i in range(8):
            A[i] = sp.Matrix( [[sp.cos(theta[i]+x[i]), - sp.sin(theta[i]+x[i])*sp.cos(alpha[i]), sp.sin(theta[i]+x[i])*sp.sin(alpha[i]), a[i]*sp.cos(theta[i]+x[i])],
                                    [sp.sin(theta[i]+x[i]), sp.cos(theta[i]+x[i])*sp.cos(alpha[i]), -sp.cos(theta[i]+x[i])*sp.sin(alpha[i]), a[i]*sp.sin(theta[i]+x[i])],
                                    [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]],
                                    [0,0,0,1]])
        return A

    def forward_symbolic(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        T0e = sp.Identity(4)
        T = [[sp.Matrix.zeros(4) for _ in range(4) ] for _ in range(8)]
        A = self.DH_matrices(q)
        T[0] = A[0]
        for i in range(1,8):
            T[i] = T[i-1] * A[i]

        T0e = T[-1]

        # # Your code ends here
        return T0e


    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()

if __name__ == "__main__":

    fk = FK()

    q_symbols = sp.symarray('q', 7)

    y = list(sp.zeros(1))
    q_mod = y + list(q_symbols)

    T0e = fk.forward_symbolic(q_mod)

    # print ("\n\nPrinting Toe\n\n")
    #
    # for i in range(4):
    #     for j in range(4):
    #         print("T0e Element [",j,"] [",i,"] - \n")
    #         print(T0e[i,j],'\n\n')

    j_v = sp.zeros(3,7)

    for i in range(7):
        j_v[0,i] = T0e[0, -1].diff(q_symbols[i])
        j_v[1,i] = T0e[1, -1].diff(q_symbols[i])
        j_v[2,i] = T0e[2, -1].diff(q_symbols[i])

    print ("\n\nPrinting Jacobian Matrix\n\n")

    for i in range (7):
        for j in range(3):
            print("J_V Element [",j,"] [",i,"] - \n")
            print(j_v[j,i],"\n\n")

    q_values = [0,0,0,0,0,]
