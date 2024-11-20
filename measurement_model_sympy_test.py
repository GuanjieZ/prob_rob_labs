from sympy import symbols, Matrix, sin, cos, atan2, simplify, lambdify
def init_euation():
    x, y, theta, t_cx, t_cy, t_cz, x_l, y_l, r_l, h_l, f_x, f_y, c_x, c_y \
        = symbols("x y theta t_cx t_cy t_cz x_l y_l r_l h_l f_x f_y c_x c_y")
        
    x_c = cos(theta)*t_cx - sin(theta)*t_cy + x
    y_c = sin(theta)*t_cx + cos(theta)*t_cy + y
    phi = atan2(y_l - y_c, x_l - x_c)

    x1 = x_l - r_l*sin(phi)
    y1 = y_l + r_l*cos(phi)
    x2 = x_l + r_l*sin(phi)
    y2 = y_l - r_l*cos(phi)

    P_1g = Matrix([[x1, y1, 0, 1]]).T
    P_2g = Matrix([[x2, y2, 0, 1]]).T
    P_3g = Matrix([[x2, y2, h_l, 1]]).T
    P_4g = Matrix([[x1, y1, h_l, 1]]).T
    P_ig = [P_1g, P_2g, P_3g, P_4g]

    T_mr = Matrix([[cos(theta), -sin(theta), 0, x],
                [sin(theta),  cos(theta), 0, y],
                [0,           0,          1, 0],
                [0,           0,          0, 1]    
                ])

    T_ro = Matrix([[ 0,  0, 1, t_cx],
                [-1,  0, 0, t_cy],
                [ 0, -1, 0, t_cz],
                [ 0,  0, 0, 1]    
                ])

    T_mo = T_mr * T_ro
    # print(T_mo.inv())
    T_om = simplify(T_mo.inv())
    # print(T_om)

    P_io = []
    for element in P_ig:
        P_io.append(T_om * element)

    Proj_mat = Matrix([[f_x, 0,   c_x],
                    [0,   f_y, c_y],
                    [0,   0,   1]
                    ])

    auxiliary_vec = []
    for element in P_io:
        auxiliary_vec.append(Proj_mat * element[:3, :])

    P_ip_list = []
    for element in auxiliary_vec:
        P_ip_list.append(element[:2, :]/element[2,0])

    P_ip = Matrix([[P_ip_list[0][0,0]],
                [P_ip_list[0][1,0]],
                [P_ip_list[1][0,0]],
                [P_ip_list[1][1,0]],
                [P_ip_list[2][0,0]],
                [P_ip_list[2][1,0]],
                [P_ip_list[3][0,0]],
                [P_ip_list[3][1,0]],
    ])

    states = Matrix([x, y, theta])
    Hx = P_ip.jacobian(states)
    Hx_func = lambdify((x, y, theta, t_cx, t_cy, t_cz, x_l, y_l, r_l, h_l, f_x, f_y, c_x, c_y), Hx)

    P_ip_func = lambdify((x, y, theta, t_cx, t_cy, t_cz, x_l, y_l, r_l, h_l, f_x, f_y, c_x, c_y), P_ip)
    
    return Hx_func, P_ip_func

Hx, P_ip = init_euation()

result = Hx(1.0, 2.0, 0.5, 0.1, 0.2, 0.3, 4.0, 5.0, 1.2, 3.3, 500, 500, 250, 250)
print("Evaluated result of Hx_func:", result)
result2 = P_ip(1.0, 2.0, 0.5, 0.1, 0.2, 0.3, 4.0, 5.0, 1.2, 3.3, 500, 500, 250, 250)
print("Evaluated result of Hx_func:", result2)
