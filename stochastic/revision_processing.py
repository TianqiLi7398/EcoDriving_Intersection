import json
import matplotlib.pyplot as plt
import os


timeset = [26, 5, 25]                      # a, red, yellow, green
timeset = [15, 3, 20]                      # b, red, yellow, green
trafficFolderName = str(timeset[0]) + '_' + str(timeset[1]) + '_' + str(timeset[2])
vel_collection = [5, 10, 15, 20]
vel_collection = [15]
density = 0.7489
plt.rc('xtick',labelsize=18)
plt.rc('ytick',labelsize=18)


for init_vel in vel_collection:
    x0 = [0, init_vel]
    # filename = str(0) + '-' + str(x0[1]) + '-' + str(timeset[0])+'-'+str(timeset[1])+'-'+str(timeset[2])+'_monte_carlo.json'
    # with open(os.path.join(os.getcwd(), 'data', filename)) as json_file:
    #     cost_table = json.load(json_file)

    filename = str(0) + '-' + str(x0[1]) + '-' + str(timeset[0])+'-'+str(timeset[1])+'-'+str(timeset[2])+'_monte_carlo.json'
    with open(os.path.join(os.getcwd(), 'data', filename)) as json_file:
        cost_table_1 = json.load(json_file)
    
    filename = str(0) + '-' + str(x0[1]) + '-' + str(timeset[0])+'-'+str(timeset[1])+'-'+str(timeset[2])+'_monte_carlo_2.json'
    with open(os.path.join(os.getcwd(), 'data', filename)) as json_file:
        cost_table_2 = json.load(json_file)
    
    cost_table = {"sto": [], "det": [], "dum": [], "opt": []}
    cost_table["sto"] = cost_table_1["sto"] + cost_table_2["sto"]
    cost_table["det"] = cost_table_1["det"] + cost_table_2["det"]
    cost_table["opt"] = cost_table_1["opt"] + cost_table_2["opt"]
    cost_table["dum"] = cost_table_1["dum"] + cost_table_2["dum"]
        
    ave_sto, ave_det, ave_dum, ave_opt = [], [], [], []
    sum_sto, sum_det, sum_dum, sum_opt = 0, 0, 0, 0
    count = 0
    for i in range(len(cost_table["sto"])):
        sum_sto += cost_table["sto"][i] * density
        sum_dum += cost_table["dum"][i] * density
        sum_det += cost_table["det"][i] * density
        if cost_table["opt"][i] > 0:
            sum_opt += cost_table["opt"][i]* density
            count += 1
        
        ave_sto.append(sum_sto / (i + 1))
        ave_det.append(sum_det / (i + 1))
        ave_dum.append(sum_dum / (i + 1))
        ave_opt.append(sum_opt / count)
    plt.figure(figsize=(15,5))
    plt.tight_layout()
    print("v = %s, sto: %s, det: %s, dum: %s, opt: %s" % (init_vel, round(ave_sto[-1], 3), round(ave_det[-1], 3), round(ave_dum[-1], 3), round(ave_opt[-1], 3)))
    plt.plot(ave_det )
    plt.plot(ave_sto)
    plt.plot(ave_dum)
    plt.plot(ave_opt)
    
    plt.legend(["SSOP", "SSSP", "human", "TSOP"], prop={"size":16})
    plt.xlabel("trail times", fontsize=18)
    plt.ylabel("Average fuel comsumption/g", fontsize=18)
    # plt.title("Monte Carlo compare of v0 = %s in %d trials" % (init_vel, len(cost_table["sto"])))
    foldername = 'v0='+ str(x0[1])
    pic_name = 'revision_v0='+ str(init_vel)+ '_Monte_Carlo.png'
    plt.savefig(os.path.join(os.getcwd(), 'pics', 'Monte_Carlo', trafficFolderName, foldername, pic_name), bbox_inches='tight')
    plt.close()