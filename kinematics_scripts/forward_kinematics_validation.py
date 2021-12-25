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

    T_0_6 = get_final_matrix(dh_params)
    print("Final Transformation for FORWARD KINEMATICS: \n", T_0_6)

    print("===========================FORWARD KINEMATICS VALIDATAION==============================")

    T1, T2, T3, T4, T5, T6 = 90, 0, 0, 0, 0, 0
    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    T_0_6 = get_final_matrix(dh_params)
    print("\nFor Joint space Theta [90, 0, 0, 0, 0, 0]")
    print("From Geometrical Method\nEnd Effector Position\nX: {}\nY: {}\nZ: {}".format(0, 0, D1 + A2 + D3 + D4 + A5))
    print("From DH Parameters\nEnd Effector Position\nX: {}\nY: {}\nZ: {}\n".format(T_0_6.col(-1)[0], T_0_6.col(-1)[1], T_0_6.col(-1)[2]))

    T1, T2, T3, T4, T5, T6 = 0, 90, 0, 0, 0, 0
    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    T_0_6 = get_final_matrix(dh_params)
    print("\nFor Joint space Theta [0, 90, 0, 0, 0, 0]")
    print("From Geometrical Method\nEnd Effector Position\nX: {}\nY: {}\nZ: {}".format(A2 + D3 + D4 + A5, 0, D1))
    print("From DH Parameters\nEnd Effector Position\nX: {}\nY: {}\nZ: {}\n".format(T_0_6.col(-1)[0], T_0_6.col(-1)[1], T_0_6.col(-1)[2]))


    T1, T2, T3, T4, T5, T6 = 0, 0, 90, 0, 0, 0
    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    T_0_6 = get_final_matrix(dh_params)
    print("\nFor Joint space Theta [0, 0, 90, 0, 0, 0]")
    print("From Geometrical Method\nEnd Effector Position\nX: {}\nY: {}\nZ: {}".format(D3 + D4 + A5, 0, D1 + A2))
    print("From DH Parameters\nEnd Effector Position\nX: {}\nY: {}\nZ: {}\n".format(T_0_6.col(-1)[0], T_0_6.col(-1)[1], T_0_6.col(-1)[2]))


    T1, T2, T3, T4, T5, T6 = 0, 0, 0, 90, 0, 0
    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    T_0_6 = get_final_matrix(dh_params)
    print("\nFor Joint space Theta [0, 0, 0, 90, 0, 0]")
    print("From Geometrical Method\nEnd Effector Position\nX: {}\nY: {}\nZ: {}".format(0, 0, D1 + A2 + D3 + D4 + A5))
    print("From DH Parameters\nEnd Effector Position\nX: {}\nY: {}\nZ: {}\n".format(T_0_6.col(-1)[0], T_0_6.col(-1)[1], T_0_6.col(-1)[2]))


    T1, T2, T3, T4, T5, T6 = 0, 0, 0, 0, 90, 0
    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    T_0_6 = get_final_matrix(dh_params)
    print("\nFor Joint space Theta [0, 0, 0, 0, 90, 0]")
    print("From Geometrical Method\nEnd Effector Position\nX: {}\nY: {}\nZ: {}".format(A5, 0, D1 + A2 + D3 + D4))
    print("From DH Parameters\nEnd Effector Position\nX: {}\nY: {}\nZ: {}\n".format(T_0_6.col(-1)[0], T_0_6.col(-1)[1], T_0_6.col(-1)[2]))


    T1, T2, T3, T4, T5, T6 = 0, 0, 0, 0, 0, 90
    dh_params = [[0, -90, D1, T1],
                 [-A2, 0, 0, T2 + 90],
                 [0, 90, 0, T3 - 90],
                 [0, -90, D3 + D4, T4],
                 [A5, 0, 0, T5 - 90],
                 [0, 0, 0, T6]]

    T_0_6 = get_final_matrix(dh_params)
    print("\nFor Joint space Theta [0, 0, 0, 0, 0, 90]")
    print("From Geometrical Method\nEnd Effector Position\nX: {}\nY: {}\nZ: {}".format(0, 0, D1 + A2 + D3 + D4 + A5))
    print("From DH Parameters\nEnd Effector Position\nX: {}\nY: {}\nZ: {}\n".format(T_0_6.col(-1)[0], T_0_6.col(-1)[1], T_0_6.col(-1)[2]))