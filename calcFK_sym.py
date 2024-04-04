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

    def DH_param(self):
        a = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        alpha = [0, -sp.pi/2, sp.pi/2, sp.pi/2, sp.pi/2, -sp.pi/2, sp.pi/2, 0]
        d = [0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.210]
        x = [0,0,0,0, sp.pi/2, 0, -sp.pi/2, 0]
        q_corr = [0,0,0,0,+sp.pi/2,0,-sp.pi/2,-sp.pi/4]
        #theta = sp.array([self.q[0], self.q[1], self.q[2], self.q[3], pi/2 + self.q[4], self.q[5], -pi/2 + self.q[6], self.q[7]])
        theta = self.q + x

        for i in range(8):
            self.A[i] = sp.Matrix([[sp.cos(theta[i]+x[i]+q_corr[i]), - sp.sin(theta[i]+x[i]+q_corr[i])*sp.cos(alpha[i]), sp.sin(theta[i]+x[i]+q_corr[i])*sp.sin(alpha[i]), a[i]*sp.cos(theta[i]+x[i]+q_corr[i])], [sp.sin(theta[i]+x[i]+q_corr[i]), sp.cos(theta[i]+x[i]+q_corr[i])*sp.cos(alpha[i]), -sp.cos(theta[i]+x[i]+q_corr[i])*sp.sin(alpha[i]), a[i]*sp.sin(theta[i]+x[i]+q_corr[i])], [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]], [0,0,0,1]])


    def forward(self, q):
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
        y = list(sp.zeros(1))
        self.q = y + list(q)
        # q1 = np.array([0,0,0,0,-pi/2,0,pi/2,pi/4])
        # self.q = self.q - q1

        # jointPositions = sp.zeros(8,3)
        T0e = sp.Identity(4)#np.identity(4)

        self.DH_param()
        self.T[0] = self.A[0].copy()
        for i in range(1,8):
            self.T[i] = self.T[i-1].copy() * self.A[i].copy()

        T0e = self.T[-1].copy()
        print(T0e.shape)
        # for i in range(8):
        #     jointPositions[i] = self.T[i][:-1, -1].copy()
        # # jointPositions = self.T[:,:-1, -1]

        # jointPositions[2] =  jointPositions[2] + 0.195 * self.T[2][:-1, :-1].copy() * np.array([0, 0, 1])
        # jointPositions[4] = jointPositions[4] + 0.125 *self.T[4] [:-1, :-1].copy() * np.array([0, 0, 1])
        # jointPositions[5] = jointPositions[5] - 0.015 * self.T[5][:-1, :-1].copy() * np.array([0, 0, 1])
        # jointPositions[6] = jointPositions[6] + 0.051 * self.T[6][:-1, :-1].copy() * np.array([0, 0, 1])
        # # Your code ends here

        return T0e

    # feel free to define additional helper methods to modularize your solution for lab 1



if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout

    #q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q = sp.symarray('q', 7)




    T0e = fk.forward(q)

    # print(fk.A)
    # print("Joint Positions:\n",joint_positions)
    # print("End Effector Pose:\n",T0e)
    # print(T0e.shape)
    # for i in range(4):
    #     for j in range(4):
    #         print(T0e[i,j],'\n')


    Jacobian = sp.zeros(7,3)
    for i in range(7):
        Jacobian[i,0] = T0e[0, -1].copy().diff(q[i])
        Jacobian[i,1] = T0e[1, -1].copy().diff(q[i])
        Jacobian[i,2] = T0e[2, -1].copy().diff(q[i])

    # print(Jacobian[0,2])
    # print(Jacobian[1,2])

    Jacobian = Jacobian.T

    # with open("C:/Users/royin/OneDrive - PennO365/Documents/IntroToRobo/linear_vel_Jacobian.txt", "w") as outf:
    #     outf.write(str(Jacobian))
    # 0,0,0,-pi/2,0,pi/2,pi/4
    # print('\n Linear Vel Jacobian \n',T0e.subs({q[0]:0, q[1]:0, q[2]:0, q[3]:-np.pi/2, q[4]:0, q[5]:np.pi/2, q[6]:np.pi/4}), "\n")
    # print('\n Robot configuration - \n', np.array([0,0,0,-np.pi/2,0,np.pi/2,np.pi/4]))
    # print('\n Linear Vel Jacobian \n',Jacobian.subs({q[0]:0, q[1]:0, q[2]:0, q[3]:-np.pi/2, q[4]:0, q[5]:np.pi/2, q[6]:np.pi/4}), "\n")
    print('\n Robot configuration - \n', np.array([ pi/2, 0,  pi/4, -pi/2, -pi/2, pi/2,    0 ]))
    print('\n Linear Vel Jacobian \n',Jacobian.subs({q[0]:np.pi/2, q[1]:0, q[2]:np.pi/4, q[3]:-np.pi/2, q[4]:-np.pi/2, q[5]:np.pi/2, q[6]:0}), "\n")

    # print('Jacobian - \n', Jacobian)
    # J_val = Jacobian.subs({q[0]:0, q[1]:0, q[2]:0, q[3]:-np.pi/2, q[4]:0, q[5]:np.pi/2, q[6]:np.pi/4})
    #
    # q_dot1 = np.array([1, 0, 0, 0, 0, 0, 0]).reshape((7,1))
    # lin_v = np.matmul(J_val, q_dot1)
    #
    # print('\n',lin_v)
    #
    # q_dot2 = np.array([0, 1, 0, 0, 0, 0, 0]).reshape((7,1))
    # lin_v = np.matmul(J_val, q_dot2)
    #
    # print('\n',lin_v)
    #
    # q_dot3 = np.array([0, 0, 1, 0, 0, 0, 0]).reshape((7,1))
    # lin_v = np.matmul(J_val, q_dot3)
    #
    # print('\n',lin_v)
