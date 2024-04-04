import numpy as np
from math import pi
from math import cos, sin

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        # DH_Params - [theta_offset, d_offset,    a,     alpha]
        self.DH_params  =  [[0,     0.141,        0,          0],
                            [0,     0.192,        0,      -pi/2],
                            [0,     0.000,        0,       pi/2],
                            [0,     0.195+0.121,  0.0825,  pi/2],
                            [pi/2,  0.000,        0.0825,  pi/2],
                            [0,     0.125+0.259,  0,      -pi/2],
                            [-pi/2, 0.000,        0.088,   pi/2],
                            [0,     0.051+0.159,  0,          0]]

        pass



    def DH_transformation(self, q_modified, joint_number):
        """
        INPUT: q_modified   -   modified vector of joint angles of the form [0, q0, q1, q2, q3, q4, q5, q6] - modification done to correct shape and inputs
               DH_params    -   vector of DH_param constants [theta_offset, d, a, alpha]
               joint_number -   which joint to calculate DH Params for
                    In DH Parameters, theta = theta_offset + q_modified (corrected joint variable) - for revolute
                    For usage here we have used only the offsets for theta (for neutral position).
                    Joint variables will be passed additionally to the function and added to offsets to formulate the correct matrices
        """

        DH = self.DH_params[joint_number]

        theta = DH[0] + q_modified[joint_number]          #theta = theta_offset + joint variable
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

        """
        q_modified is created to be a 8x1 vector which is bascially the q(input) with a 0 appended at the 0th index.
        This is done to make the shape of the input similar to that of the output (8x3 - jointPosition)
        q_correction is subtracted from this value because the input of our code gives the position in which DH was computed.
        But the input that needs to be given to achieve said position is actually [0,0,0,-pi/2,0,pi/2,pi/4] - given in image of franka
        q_modified is the final vector that can be used to compute transformations
        """
        q_modified = np.insert(q, 0 ,0)

        q_correction=np.array((8,1))
        q_correction = [0,0,0,0,-pi/2,0,pi/2,pi/4]

        q_modified = np.subtract(q_modified, q_correction)

        A = np.zeros((8,4,4))
        """
        creating a 3D matrix A - which is basically 8 - 4x4matrices stacked (8x4x4).
        Each 4x4 matrix is the transfromation matrix from the previous intermidiate frame
        """

        for i in range(0,8):
            A[i] = self.DH_transformation(q_modified, i)
            # print(A[i], '\n')

        """
        Create a 3D matrix which is basically a stack of the transformation matrices of intermidiate frames wrt 0
        T_1wrt0 = A[0]
        T_2wrt0 = T[0] = A[0]*A[1]
        T_3wrt0 = T[1] = T[0]*A[2]
        T_4wrt0 = T[2] = T[0]*A[3]
        T_5wrt0 = T[3] = T[0]*A[4]
        T_6wrt0 = T[4] = T[0]*A[5]
        T_7wrt0 = T[5] = T[0]*A[6]
        T_Ewrt0 = T[6] = T[0]*A[7]      (EndEffector Frame)
        """

        T_wrt_0 = np.zeros((7,4,4))
        T_wrt_0[0] = np.dot(A[0], A[1])

        for j in range(1,7):
            T_wrt_0[j] = np.dot(T_wrt_0[j-1], A[j+1])

        """
        Defined an array comprising of offsets from respective local origins - All offsets are wrt z axis in this case
        jointPositions is 8x3 vector comprising of coordinates of each joint wrt base frame - 7 joints + 1 EndEff frame

        """

        jointPositions = np.zeros((8,3))

        offsetFromLocalOrigin = np.array([0, 0, 0.195, 0, 0.125, -0.015, 0.051, 0])

        jointPositions[0] = A[0,:-1,3] + offsetFromLocalOrigin[0]

        for i in range (1,8):
            jointPositions[i] = T_wrt_0[i-1,:-1,3] + offsetFromLocalOrigin[i]*(np.dot(T_wrt_0[i-1,:-1,:-1], np.array([0,0,1])))
        # jointPositions[1] = T_wrt_0[0,:-1,3]
        # jointPositions[2] = T_wrt_0[1,:-1,3] + 0.195*(np.dot(T_wrt_0[1,:-1,:-1], np.array([0,0,1])))
        # jointPositions[3] = T_wrt_0[2,:-1,3]
        # jointPositions[4] = T_wrt_0[3,:-1,3] + 0.125*(np.dot(T_wrt_0[3,:-1,:-1], np.array([0,0,1])))
        # jointPositions[5] = T_wrt_0[4,:-1,3] + 0.015*(np.dot(T_wrt_0[4,:-1,:-1], np.array([0,0,-1])))
        # jointPositions[6] = T_wrt_0[5,:-1,3] + 0.051*(np.dot(T_wrt_0[5,:-1,:-1], np.array([0,0,1])))
        # jointPositions[7] = T_wrt_0[6,:-1,3]

        T0e = np.identity(4)
        T0e = T_wrt_0[6]

        # Your code ends here

        return jointPositions, T0e, T_wrt_0

    # feel free to define additional helper methods to modularize your solution for lab 1


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

    # matches figure in the handout
    q = np.array([-1.78449, -1.43698, -0.42498, -2.39348, -0.49742,  1.02813,  0.23803])
    # q = np.array([0,0,0,0,0,0,0])

    joint_positions, T0e, T = fk.forward(q)

    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
    # print("Intermidiate Ts Pose:\n",T)
