import numpy as np

a_minmax = [-5, 8]

v_list = np.linspace(0, 22, 23)

dt = 2

for v in v_list:
    dv_list = v_list - v
    print("v = %d" % v)
    for dv in dv_list:
        a = dv/dt 
        if a <= a_minmax[1] and a >= a_minmax[0]:
            x = v * dt + 0.5 * dv * dt**2
            print("dv = %d, a = %f, x = %f" % (dv, a, x))