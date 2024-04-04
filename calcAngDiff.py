import numpy as np


def calcAngDiff(R_des, R_curr):
    """
    Helper function for the End Effector Orientation Task. Computes the axis of rotation
    from the current orientation to the target orientation

    This data can also be interpreted as an end effector velocity which will
    bring the end effector closer to the target orientation.

    INPUTS:
    R_des - 3x3 numpy array representing the desired orientation from
    end effector to world
    R_curr - 3x3 numpy array representing the "current" end effector orientation

    OUTPUTS:
    omega - 0x3 a 3-element numpy array containing the axis of the rotation from
    the current frame to the end effector frame. The magnitude of this vector
    must be sin(angle), where angle is the angle of rotation around this axis
    """
    omega = np.zeros(3)
    ## STUDENT CODE STARTS HERE

    Rot_currToDes = np.dot(np.linalg.inv(R_curr),R_des)
    skew = 0.5 * (Rot_currToDes - np.transpose(Rot_currToDes))
    a_x = skew[2][1]
    a_y = skew[0][2]
    a_z = skew[1][0]
    Rot_vect = np.array([a_x, a_y, a_z])
    Rot_vect_wrt0 = np.dot(R_curr, Rot_vect)
    omega = Rot_vect_wrt0
    # print (omega)
    return omega

if __name__ == '__main__':

    # R_des = np.array([[1,0,0],[0,1,0],[0,0,1]])
    R_des = np.array([[ 0.85286853, -0.17411041,0.49385022],
                      [ 0.49385022, 0.22807134,-0.83908446],
                      [ 0.17411041, 0.95781246, 0.22653852]])
    R_curr = np.array([[ 0.73670382, -0.59999489,  0.31324965],
                       [ 0.31324965,  0.09515447, -0.94514557],
                       [ 0.59999489,  0.79482701,  0.08801847]])

    print ("Initial Config - ", R_curr, "\n")
    print ("Desired Config - ", R_des, "\n")
    print("Angle Diff: \n", calcAngDiff(R_des, R_curr))
