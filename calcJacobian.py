import numpy as np
from lib.calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((7,6))
    Jv = np.zeros((7,3))
    Jw = np.zeros((7,3))

    ## STUDENT CODE GOES HERE

    fk = FK()
    # print("Q_IN IN CALCJACOBIAN.PY", q_in)
    # print("SHAPE OF Q_IN IN CALCJACOBIAN.PY", q_in.shape)
    joint_positions, T0e, T = fk.forward(q_in)

    # print (T0e[0:3,-1])
    # print (T[0][0:3,-1])
    Jv[0] = np.cross([0,0,1], T0e[0:3,-1])
    Jw[0] = np.dot([0,0,1], 1)
    J[0] = np.append(Jv[0],Jw[0])
    # print(J[0])
    for i in range (1,7):
        Jv[i] = np.cross(T[i-1][0:3,2], (T0e[0:3,-1] - T[i-1][0:3,-1]))
        Jw[i] = np.dot(T[i-1][0:3,2], 1)
        J[i] = np.append(Jv[i],Jw[i])


    # print (J)
    J = np.transpose(J)
    # print (J)
    return J

if __name__ == '__main__':
    # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q = np.array([ np.pi/2, 0,  np.pi/4, -np.pi/2, -np.pi/2, np.pi/2,    0 ])
    # print(np.shape(calcJacobian(q)))
    print("Robot configuration - \n", q, '\n')
    J = calcJacobian(q)
    print("Jacobian(v) - \n", J[0:3,:])
    # A = np.zeros((3,1))
    # print(A)
