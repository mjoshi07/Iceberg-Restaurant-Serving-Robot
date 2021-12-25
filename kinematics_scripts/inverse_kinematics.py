import sympy.core
from sympy import *
import numpy as np

DEG_2_RAD = np.pi/180.0
RAD_2_DEG = 1/DEG_2_RAD


def get_angles(angle, prefix=''):
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
        cos_angle = round(np.cos(angle*DEG_2_RAD), 2)
        sin_angle = round(np.sin(angle*DEG_2_RAD), 2)

    return cos_angle, sin_angle


def get_val(val):
    if isinstance(val, str):
        return symbols(val)
    else:
        return val


def get_transformation_matrix(dh_parameters):
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
    T_i = Matrix([[cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
                  [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
                  [0, sin_alpha, cos_alpha, d],
                  [0, 0, 0, 1]])
    return T_i.evalf(1)


def get_final_matrix(dh_parameters, params_idx=None):
    identity_matrix = Matrix([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
    final_matrix = identity_matrix
    if params_idx is None:
        for params in dh_parameters:
            final_matrix *= get_transformation_matrix(params)
    else:
        for idx in range(params_idx+1):
            final_matrix *= get_transformation_matrix(dh_parameters[idx])

    return final_matrix.evalf(1)


def get_end_effector_vector(matrix_0_i=None):
    if matrix_0_i is None:
        return Matrix([[0], [0], [0]])
    else:
        return Matrix([[matrix_0_i.col(-1)[0]], [matrix_0_i.col(-1)[1]], [matrix_0_i.col(-1)[2]]])


def get_z_vector(matrix_0_i=None):
    if matrix_0_i is None:
        return Matrix([[0], [0], [1]])
    else:
        return Matrix([[matrix_0_i.col(-2)[0]], [matrix_0_i.col(-2)[1]], [matrix_0_i.col(-2)[2]]])


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


if __name__ == "__main__":

    D1, D3, D4 = symbols('d1 d3 d4')
    T1, T2, T3, T4, T5, T6 = symbols('θ1 θ2 θ3 θ4 θ5 θ6')

    A2, A5 = symbols('a2 a5')

    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    jacobian_matrix = get_jacobian(dh_params)

    print("Jacobian matrix \n", jacobian_matrix)