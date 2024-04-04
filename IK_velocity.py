import numpy as np
from lib.calcJacobian import calcJacobian



def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE

    dq = np.zeros((1, 7))

    v_in = v_in.reshape((1,3))
    omega_in = omega_in.reshape((1,3))

    J = calcJacobian(q_in)      #shape = 6,7

    velocity = np.append(v_in, omega_in)        # shape = 1,6
    nan_check = np.isnan(velocity)              # checks at what indices velocity is nan: returns a list of true/false for corrosponding index
    velocity = np.reshape(velocity,(6,1))       #shape = 6,1

    rows_to_delete = np.where(nan_check == True)

    #Delete rows of J and velocity where velocity is nan
    velocity_mod = np.delete(velocity,rows_to_delete,0)
    J_mod = np.delete(J,rows_to_delete,0)

    dq,_,_,_ = np.linalg.lstsq(J_mod, velocity_mod, rcond=None)

    return dq, J_mod

if __name__ == '__main__':
    q_in= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    v_in = np.array([0,1,0])
    omega_in = np.array([np.nan,np.nan,np.nan])

    dq = np.zeros((7,1))
    dq = IK_velocity(q_in, v_in, omega_in)
    print(dq, "\n")
