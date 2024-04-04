import numpy as np
from lib.calcJacobian import calcJacobian

def calcManipulability(q_in):
    """
    Helper function for calculating manipulability ellipsoid and index

    INPUTS:
    q_in - 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]

    OUTPUTS:
    mu - a float scalar, manipulability index
    M  - 3 x 3 manipulability matrix for the linear portion
    """
    J = calcJacobian(q_in)

    J_pos = J[:3,:]
    M = J_pos @ J_pos.T

    _, S, _ = np.linalg.svd(J_pos)
    print(S)

    ## STUDENT CODE STARTS HERE for the mu index, Hint: np.linalg.svd
    mu = S[0]*S[1]*S[2]

    return mu, M


if __name__ == '__main__':
    q = np.array([ np.pi/2, 0,  np.pi/4, -np.pi/2, -np.pi/2, np.pi/2,    0 ])
    # R_des = np.array([[1,0,0],[0,1,0],[0,0,1]])
    calcManipulability(q)
