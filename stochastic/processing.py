import pandas as pd
import matplotlib.pyplot as plt
import os


timeset = [26, 5, 25]                      # a, red, yellow, green
# timeset = [15, 3, 20]                      # b, red, yellow, green
trafficFolderName = str(timeset[0]) + '_' + str(timeset[1]) + '_' + str(timeset[2])
vel_collection = [5, 10, 15, 20]
dt = 0.01

for init_vel in vel_collection:
    x0 = [0, init_vel]
    filename = str(0) + '-' + str(x0[1]) + '-' + str(timeset[0])+'-'+str(timeset[1])+'-'+str(timeset[2])+'_monte_carlo.csv'
    cost_table = pd.read_csv(os.path.join(os.getcwd(), 'data', filename), sep='\t') 
    ave_sto, ave_det, ave_dum = [], [], []
    sum_sto, sum_det, sum_dum = 0, 0, 0
    for i in range(len(cost_table)):
        sum_sto += cost_table["sto"][i] 
        sum_dum += cost_table["dum"][i] 
        sum_det += cost_table["det"][i] 
        ave_sto.append(sum_sto / (i + 1))
        ave_det.append(sum_det / (i + 1))
        ave_dum.append(sum_dum / (i + 1))
    print("v = %s, sto: %s, det: %s, dum: %s" % (init_vel, ave_sto[-1], ave_det[-1], ave_dum[-1]))
    plt.plot(ave_det)
    plt.plot(ave_sto)
    plt.plot(ave_dum)
    plt.legend(["det", "sto", "smart"])
    plt.xlabel("trail times")
    plt.ylabel("Average fuel comsumption/ 1e-3 Liter")
    plt.title("Monte Carlo compare of v = %s in %d times" % (init_vel, len(cost_table)))
    foldername = 'v0='+ str(x0[1])
    pic_name = 'v0='+ str(init_vel)+ '_Monte_Carlo.png'
    plt.savefig(os.path.join(os.getcwd(), 'pics', 'Monte_Carlo', trafficFolderName, foldername, pic_name))
    plt.close()