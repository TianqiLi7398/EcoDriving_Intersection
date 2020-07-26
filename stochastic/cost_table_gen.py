import pandas as pd
import numpy as np
import os

def FC(v, a): 
    # Speed v: m/s
    # acceleration a: m/s^2
    # output: originally is L/s, change it to cm^3/s for computation convinence 
    vel=float(v)
    vel = vel * 3.6 # convert m/s -> km/h
    acc=float(a)
    Chig = 1
    Cr = 1.75 
    c1 = 0.0328
    C2 = 4.575
    Afront = 2.28
    etaD = 0.92
    m = 1470; # Kg
    Cdrag = 0.28
    widle = 700; # (rpm)
    d = 2.5
    N = 4
    FEhigh = 35
    FEcity=25
    #Fhwy=float(38.6013)*float((1.3466/float(FEhigh))+0.001376)
    #G=0 #grade
    #Calculate the total Resistance forces.
    R = float((1.2256 / 25.92) *float(Cdrag) * float(Chig) *float(Afront) * (vel*vel) +
    (9.8066 *float(m) * float(Cr) *( (float(c1) * vel +float(C2)) / 1000)))
    #Input the calibrated parameters values.
    alpha0 = 0.0006289
    alpha1 = 2.676e-05
    alpha2 = 1e-06
    P =float(((R + 1.04 * m * acc) / (3600 * etaD)) * vel)
    #alpha0 = float((Pmf0 * widle * d * 3.42498) / (22164 * Q * N))

    Fuel_Consumption =(alpha0 + alpha1 * P + alpha2 * (P *P))
    if P < 0:
        Fuel_Consumption = float(alpha0)
    #print(alpha0,alpha1,alpha2)
    return Fuel_Consumption * 1000

def cost_fuel(v, a, delta_t):
    dt = 0.01
    length = int(delta_t//dt)
    # print(length)
    # time = np.linspace(dt, length *dt, length)
    cost = 0
    for i in range(length):
        cost += FC(v, a) * dt
        v += a * dt
    return cost

def main():
    a_min = -5  # m/s^2
    a_max = 8  # m/s^2
    x = 10   # m
    table = pd.DataFrame(columns=["v", "dv", "cost"])
    dv = 1  # m/s2
    v_grid = np.linspace(0, 22, 23)
    for v1 in v_grid:
        dv_grid = v_grid - v1
        for dv in dv_grid:
            if v1 == 0 and dv == 0:
                continue
            delta_t = 2 * x / (2 * v1 + dv)
            a = dv / delta_t
            if a >= a_min and a <= a_max:
                cost = cost_fuel(v1, a, delta_t)
                
                conn = {
                    "v": int(v1),
                    "dv": int(dv),
                    "cost": cost
                }
                table = table.append(conn, ignore_index=True)
    table.to_csv(os.path.join(os.getcwd(), 'data', 'cost_fix_dis.csv'))

if __name__ == "__main__":
    main()