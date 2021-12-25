import numpy as np
import sympy.core
from sympy import *
import matplotlib.pyplot as plt

T1, T2, T3, T4, T5, T6 = 0.01, 45, 0.01, 0.01, 0.01, 0.01
D1, D3, D4, A2, A5 = 1, 1, 1, 1, 1

DH_PARAMS = [[0, -90, D1, T1],
             [-A2, 0, 0, T2 + 90],
             [0, 90, 0, T3 - 90],
             [0, -90, D3 + D4, T4],
             [A5, 0, 0, T5 - 90],
             [0, 0, 0, T6]]


DEG_2_RAD = np.pi / 180.0
RAD_2_DEG = 1 / DEG_2_RAD

TIME_2_DRAW = 5  # total time in which circle is to be drawn in seconds
TOTAL_ANGLE = 360  # degrees
D_THETA = DEG_2_RAD * TOTAL_ANGLE / TIME_2_DRAW  # angular speed
ANGULAR_STEP = 5  # get coordinate at every 360/10 degrees
TIME_STEP = (TIME_2_DRAW * ANGULAR_STEP) / TOTAL_ANGLE

INITIAL_X = 0.7*A2 + 0.7*D3 + 0.7*D4 + 0.7*A5  # starting x coordinate in meters
INITIAL_Y = 0  # starting y coordinate in meters
INITIAL_Z = D1 + 0.7*A2 + 0.7*D3 + 0.7*D4 + 0.7*A5  # starting z coordinate in meters

RADIUS = 0.7*A2 + 0.7*D3 + 0.7*D4 + 0.7*A5  # radius of the circle to be drawn in meters

# except for x and z axis, no change in other axis therefore derivative is ZERO
Z_VEL = ROLL_VEL = PITCH_VEL = YAW_VEL = 0



def get_val(val):
    """
    :param val: could be a string or a symbol
    :return: a symbol is input is a string else returns the val
    """
    if isinstance(val, str):
        return symbols(val)
    else:
        return val


def get_angles(angle, prefix=''):
    """
    :param angle: could be string or a symbol or value
    :param prefix: some number like 1, 2, 3 representing that specific angle
    :return: a symbolic representation of angles in cos and sin function
    """
    if isinstance(angle, str):
        c = 'c' + prefix + str(angle)
        s = 's' + prefix + str(angle)
        cos_angle = symbols(c)
        sin_angle = symbols(s)
    elif isinstance(angle, sympy.Basic):
        if len(angle.args) > 1:
            if angle.args[0] == 90:
                cos_angle = sin(-angle.args[1])
                sin_angle = cos(angle.args[1])
            else:
                cos_angle = cos(angle)
                sin_angle = sin(angle)
        else:
            cos_angle = cos(angle)
            sin_angle = sin(angle)
    else:
        cos_angle = np.cos(angle * DEG_2_RAD)
        sin_angle = np.sin(angle * DEG_2_RAD)

    return cos_angle, sin_angle


def get_transformation_matrix(dh_parameters):
    """
    :param dh_parameters:  [a, α, d, θ]
    :return: transformation A matrix for the specific dh parameters
    """
    # get values from dh params list
    a = get_val(dh_parameters[0])
    alpha = dh_parameters[1]
    d = get_val(dh_parameters[2])
    theta = dh_parameters[3]

    # assign symbols to cos and sin angles
    cos_alpha, sin_alpha = get_angles(alpha, 'a')
    cos_theta, sin_theta = get_angles(theta)

    """
    Ai = Rot(z, θi).Trans(z, di).Trans(x, ai).Rot(x,αi)
    """
    # get transformation matrix Ai
    T_i = Matrix([[cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
                  [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
                  [0, sin_alpha, cos_alpha, d],
                  [0, 0, 0, 1]])
    return T_i.evalf(1)


def get_z_vector(matrix_0_i=None):
    """
    :param matrix_0_i: could be a matrix or None
    :return: first 3 elements of the third column of the matrix if not None, else returns a default value
    """
    if matrix_0_i is None:
        return Matrix([[0], [0], [1]])
    else:
        return Matrix([[matrix_0_i.col(-2)[0]], [matrix_0_i.col(-2)[1]], [matrix_0_i.col(-2)[2]]])


def get_end_effector_vector(matrix_0_i=None):
    """
    :param matrix_0_i: could be a matrix or None
    :return: first 3 elements of the last column of the matrix if not None, else returns a default value
    """
    if matrix_0_i is None:
        return Matrix([[0], [0], [0]])
    else:
        return Matrix([[matrix_0_i.col(-1)[0]], [matrix_0_i.col(-1)[1]], [matrix_0_i.col(-1)[2]]])


def get_jacobian(dh_params):
    """
    :param dh_params:  [a, α, d, θ] full dh parameter table
    :return: jacobidan matrix for the dh paramters table
    """

    # calculate individual transformation matrices
    T_0_1 = get_transformation_matrix(dh_params[0])
    T_1_2 = get_transformation_matrix(dh_params[1])
    T_2_3 = get_transformation_matrix(dh_params[2])
    T_3_4 = get_transformation_matrix(dh_params[3])
    T_4_5 = get_transformation_matrix(dh_params[4])
    T_5_6 = get_transformation_matrix(dh_params[5])


    # calculate transformation with the base frame
    T_0_2 = T_0_1 * T_1_2
    T_0_3 = T_0_2 * T_2_3
    T_0_4 = T_0_3 * T_3_4
    T_0_5 = T_0_4 * T_4_5
    T_0_6 = T_0_5 * T_5_6


    # calculate Z vector
    Z_0_0 = get_z_vector(None)
    Z_0_1 = get_z_vector(T_0_1)
    Z_0_2 = get_z_vector(T_0_2)
    Z_0_3 = get_z_vector(T_0_3)
    Z_0_4 = get_z_vector(T_0_4)
    Z_0_5 = get_z_vector(T_0_5)


    # calculate end-effector vector
    EF_0_0 = get_end_effector_vector(None)
    EF_0_1 = get_end_effector_vector(T_0_1)
    EF_0_2 = get_end_effector_vector(T_0_2)
    EF_0_3 = get_end_effector_vector(T_0_3)
    EF_0_4 = get_end_effector_vector(T_0_4)
    EF_0_5 = get_end_effector_vector(T_0_5)
    EF_0_6 = get_end_effector_vector(T_0_6)


    # angular velocity components
    Jw = [Z_0_0, Z_0_1, Z_0_2, Z_0_3, Z_0_4, Z_0_5]

    # linear velocity components
    Jv1 = Z_0_0.cross(EF_0_6 - EF_0_0)  # cross product of Z_0_0 with (On - O0)
    Jv2 = Z_0_1.cross(EF_0_6 - EF_0_1)  # cross product of Z_0_1 with (On - O1)
    Jv3 = Z_0_2.cross(EF_0_6 - EF_0_2)  # cross product of Z_0_2 with (On - O2)
    Jv4 = Z_0_3.cross(EF_0_6 - EF_0_3)  # cross product of Z_0_3 with (On - O3)
    Jv5 = Z_0_4.cross(EF_0_6 - EF_0_4)  # cross product of Z_0_4 with (On - O4)
    Jv6 = Z_0_5.cross(EF_0_6 - EF_0_5)  # cross product of Z_0_5 with (On - O5)

    Jv = [Jv1, Jv2, Jv3, Jv4, Jv5, Jv6]

    Jacobian = Matrix([Jv, Jw])

    return Jacobian


def get_inverse_mat(matrix):
    """
    :param matrix: could be a square or non-square matrix
    :return: pseudo inverse of the matrix
    """
    return matrix.pinv()


def update_joint_space(q_prev, current_q_dot):
    """
    :param q_prev: previous joint state value
    :param current_q_dot: current joint state velocity value
    :return current joint state value and updated previous joint state value

    Implemented the formula
    q_current= q_pervious + 𝑞̇_current . ∆t
    """
    q_curr = q_prev + current_q_dot * TIME_STEP * RAD_2_DEG
    q_prev = q_curr

    return q_curr, q_prev


def validate_inverse_kinematics():
    """
    :param dh_params: [a, α, d, θ] full dh parameters table
    :return: Void
    """

    # set initial joint angle values
    prev_q1 = DH_PARAMS[0][3]
    prev_q2 = DH_PARAMS[1][3]
    prev_q3 = DH_PARAMS[2][3]
    prev_q4 = DH_PARAMS[3][3]
    prev_q5 = DH_PARAMS[4][3]
    prev_q6 = DH_PARAMS[5][3]

    jacobians_and_joint_vel = []

    dh_params = DH_PARAMS

    """
    perform INVERSE KINEMATICS based on the desired end effector position and calculated inverse jacobian
    """
    for i in range(0, int(TOTAL_ANGLE), ANGULAR_STEP):
        # calculate jacobian for the DH parameters
        jacobian_matrix = get_jacobian(dh_params)

        """
        In polar coordinates, 
        x = rcos(theta)
        y = rsin(theta)

        x_vel = d/dt(x) = -rsin(theta)*d/dt(theta)
        y_vel = d/dt(y) = rcos(theta)*d/dt(theta)

        In our case we replace y by z as y is constant and we are drawing on x-z plane
        """
        x_vel = -RADIUS * np.sin(DEG_2_RAD * i) * D_THETA
        y_vel = RADIUS * np.cos(DEG_2_RAD * i) * D_THETA

        vel_matrix = Matrix([[x_vel], [y_vel], [Z_VEL], [ROLL_VEL], [PITCH_VEL], [YAW_VEL]])

        try:
            # calculate inverse of jacobian matrix
            jacobian_inverse = get_inverse_mat(jacobian_matrix)

            # calculate joint velocity matrix
            joint_vel_matrix = jacobian_inverse * vel_matrix

            # calculate theta1
            curr_q1, prev_q1 = update_joint_space(prev_q1, joint_vel_matrix[0])

            # calculate theta2
            curr_q2, prev_q2 = update_joint_space(prev_q2, joint_vel_matrix[1])

            # calculate theta4
            curr_q3, prev_q3 = update_joint_space(prev_q3, joint_vel_matrix[2])

            # calculate theta4
            curr_q4, prev_q4 = update_joint_space(prev_q4, joint_vel_matrix[3])

            # calculate theta5
            curr_q5, prev_q5 = update_joint_space(prev_q5, joint_vel_matrix[4])

            # calculate theta6
            curr_q6, prev_q6 = update_joint_space(prev_q6, joint_vel_matrix[5])

            # update DH parameters according to the new joint angles values
            dh_params = [[0, -90, D1, curr_q1],
                         [-A2, 0, 0, curr_q2+90],
                         [0, 90, 0, curr_q3-90],
                         [0, -90, D3 + D4, curr_q4],
                         [A5, 0, 0, curr_q5-90],
                         [0, 0, 0, curr_q6]]

            # save the jacobian and the joint velocity matrix to calculate the coordinates and draw the circle
            jacobians_and_joint_vel.append([jacobian_matrix, joint_vel_matrix])
        except Exception as e:
            print(e)

    # since all values are in meters we set limit of the plot so that we get a nice circle
    plt.figure(figsize=(10, 10))

    # set prev coordinate values to the initial values
    prev_x = INITIAL_X
    prev_y = INITIAL_Y

    """
    perform FORWARD KINEMATICS based on the calculated joint velocities and jacobian
    """
    for item in jacobians_and_joint_vel:
        jacobian = item[0]
        joint_velocity = item[1]

        # calculate end effector velocity
        end_effector_velocity = jacobian * joint_velocity

        # calculate current X position based on previous X coordinate and current X velocity
        curr_x = prev_x + end_effector_velocity[0] * TIME_STEP
        prev_x = curr_x

        # calculate current Z position based on previous Z coordinate and current Z velocity
        curr_y = prev_y + end_effector_velocity[1] * TIME_STEP
        prev_y = curr_y

        # generates a scatter plot
        plt.scatter(curr_x, curr_y)
        plt.pause(0.001)

    plt.show()


if __name__ == "__main__":

    validate_inverse_kinematics()
