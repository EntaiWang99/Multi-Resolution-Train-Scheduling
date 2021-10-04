'''
    Drawing Space-time Network
    Author: Entai Wang (https://github.com/EntaiWang99) 
            State Key Lab of Rail Traffic Control & Safety, Beijing Jiaotong University
    Email: entaiwang@bjtu.edu.cn
    2020/11/03
''' 

import os
import csv
import matplotlib.pyplot as plt
import numpy as np

# Input Paramter
# os.chdir("./H_1U1D_Fixed/macro_network")
# os.chdir("./H_1U1D_Flexible/macro_network")
# os.chdir("./H_1U2D_Fixed/macro_network")
# os.chdir("./H_1U2D_Flexible/macro_network")
# os.chdir("./H_2U1D_Fixed/macro_network")
# os.chdir("./H_2U1D_Flexible/macro_network")
os.chdir("./H_2U2D_Fixed/macro_network")
# os.chdir("./H_2U2D_Flexible/macro_network")

# os.chdir("./NH_1U1D_Fixed/macro_network")
# os.chdir("./NH_1U1D_Flexible/macro_network")
# os.chdir("./NH_1U2D_Fixed/macro_network")
# os.chdir("./NH_1U2D_Flexible/macro_network")
# os.chdir("./NH_2U1D_Fixed/macro_network")
# os.chdir("./NH_2U1D_Flexible/macro_network")
# os.chdir("./NH_2U2D_Fixed/macro_network")
# os.chdir("./NH_2U2D_Flexible/macro_network")

MAX_AGENT_NO = 6
g_number_of_stations = 3
DownStream = [2,3,4,5]
UpStream = [0,1]
MAX_TIME_INTERVAL = 60 * 60
g_number_of_iterations = 8

class Agent:
    def __init__(self):
        self.agent_no = 0
        self.node_list = []
        self.time_list = []

g_agent_list = []
station_list = ['W6-B','W6-1','W6-2','W6-III','W6-IV','W6-5', 'W6-A', 
                # 'W7-B','W7-1','W7-II','W7-III','W7-4','W7-A',   # 1U1D
                # 'W7-B','W7-0','W7-1','W7-II','W7-III','W7-4','W7-A',  # 1U2D
                # 'W7-B','W7-1','W7-II','W7-III','W7-4','W7-5','W7-A',  # 2U1D
                'W7-B','W7-0','W7-1','W7-II','W7-III','W7-4','W7-5','W7-A',  # 2U2D

                'W8-B','W8-1','W8-II','W8-3','W8-IV','W8-5', 'W8-A']

station_coord = [0,1,2,3,4,5,6,
                #  11,13,14,15,16,18, # 1U1D               
                #   11,12,13,14,15,16,18,  #1U2D
                #   11,13,14,15,16,17,18, #2U1D
                  11,12,13,14,15,16,17,18, # 2U2D
                 23,24,25,26,27,28,29
]
station_map = {}

color_value = {
        0: 'midnightblue', 
        1: 'mediumblue', 
        2:'c',
        3:'orangered',
        4:'m',
        5:'fuchsia',
        6:'olive',
        7:'tan',
        8:'black',
        9:'purple',
        10:'blue',
        11:'blueviolet',
        12:'brown',
        13:'burlywood',
        14:'cadetblue',
        15:'dimgray',
        16:'chocolate',
        17:'coral',
        18:'cornflowerblue',
        19:'fuchsia',
        20:'crimson',
        21:'cyan',
        22:'darkblue',
        23:'darkcyan',
        24:'darkgoldenrod',
        25:'darkgray',
        26:'darkgreen',
        27:'darkkhaki',
        28:'darkmagenta',
        29:'darkolivegreen',
        30:'darkorange',
        31:'darkorchid',
        32: 'midnightblue', 
        33: 'mediumblue', 
        34:'c',
        35:'orangered',
        36:'m',
        37:'fuchsia',
        38:'olive',
        39:'tan',
        40:'black',
        41:'purple',
        42:'blue',
        43:'blueviolet',
        44:'brown',
        45:'burlywood',
        46:'cadetblue',
        47:'dimgray',
    }


g_iteration_list = []
g_upper_bound_list = []
g_lower_bound_list = []
g_unfeasible_agent_list = []


if __name__ == '__main__':

    # Step 1: Drawing Timetable 
    file = 'output_drawing_agent_file.csv'
    csv_file = open(file,'rU') 
    csv_reader = csv.reader(csv_file)

    for i in range(0, MAX_AGENT_NO):
        g_agent_list.append(i)
        g_agent_list[i] = Agent()

    station_map = dict(zip(station_list, station_coord))

    for one_line in csv_reader:
        if (one_line[0]=='agent_no'):
            continue
        agent_no = int(one_line[0])
        g_agent_list[agent_no].agent_no = agent_no
        if(one_line[1] in station_list):
            g_agent_list[agent_no].node_list.append(station_map[one_line[1]])
            g_agent_list[agent_no].time_list.append(int(one_line[2]))

    for agent in g_agent_list:
        if agent.agent_no in DownStream:
            agent.node_list.sort(reverse = False)
            agent.time_list.sort(reverse = True)
        if agent.agent_no in UpStream:
            agent.node_list.sort(reverse = True)
            agent.time_list.sort(reverse = True)

    
    g_best_upper_bound = 0
    # plot_agent_no = [4,5,0,1,2,3] # for H 
    # plot_agent_no = [5,4,2,0,3,1] # for NH
    plot_agent_no = [0,1,2,3,4,5] # for default
    for agent in g_agent_list:
        xlist = agent.time_list
        ylist = agent.node_list
        if(len(xlist) != 0):
            plt.plot(xlist, ylist, color = color_value[agent.agent_no], linewidth = 1.2)
            if agent.agent_no in DownStream:
                plt.text(xlist[-1], ylist[-1] + 0.1, '%d' % int(plot_agent_no[agent.agent_no]), ha='center', va= 'bottom', \
                    color = color_value[agent.agent_no], weight = 'bold', family = 'Times new roman', fontsize= 12)
            if agent.agent_no in UpStream:
                plt.text(xlist[0], ylist[0] + 0.1, '%d' % int(plot_agent_no[agent.agent_no]), ha='center', va= 'bottom', \
                    color = color_value[agent.agent_no], weight = 'bold', family = 'Times new roman', fontsize= 12)
            g_best_upper_bound = g_best_upper_bound + max(agent.time_list) - min(agent.time_list)
    
    print("Best value: " + str(g_best_upper_bound))
    plt.grid(True) 
    plt.ylim(0, max(station_coord))

    plt.xlim(0, MAX_TIME_INTERVAL)
    plt.xticks([600 * i for i in range(int(MAX_TIME_INTERVAL/600))] , family = 'Times new roman', fontsize = 12)
 

    plt.yticks(station_coord, station_list, family = 'Times new roman', fontsize = 12)
    from pylab import mpl
    # mpl.rcParams['font.sans-serif'] = ['SimHei']
    # plt.xlabel('时间（秒）', fontsize = 12)
    # plt.ylabel('车站/股道', fontsize = 12)
    plt.xlabel('Time (second)', fontsize = 12, family = 'Times new roman')
    plt.ylabel('Station/Track', fontsize = 12, family = 'Times new roman')
    plt.show()





    # Step 2: Drawing convergence curve
    g_iteration_list = [i+1 for i in range(0, g_number_of_iterations)]
    g_upper_bound_list = []
    g_lower_bound_list = []
    g_unfeasible_agent_list = []
    g_conflict_matrix = []
    
    plt.clf
    file = 'conflct.csv'
    csv_file = open(file,'rU') 
    csv_reader = csv.reader(csv_file)

    for one_row in csv_reader:
        if (one_row[0]=='W6'):
            continue
        one_row_list = np.array(one_row).astype(dtype=float).tolist()
        g_conflict_matrix.append(one_row_list)

    for station_no in range(0, g_number_of_stations):
        xlist = g_iteration_list
        ylist = []
        for i in range(0, g_number_of_iterations):
            ylist.append(g_conflict_matrix[i][station_no] + 0.05 * station_no)
        
        # plt.plot(xlist, ylist, color = color_value[station_no], linewidth = 1.2, label='W'+str(station_no+1)+'车站')
        plt.rc('font',family='Times New Roman') 
        plt.plot(xlist, ylist, color = color_value[station_no], linewidth = 1.2, label='Station W' + str(station_no+1))
        plt.scatter(xlist, ylist, color = color_value[station_no], linewidth = 1.2)
 
    plt.xticks(family = 'Times new roman', fontsize = 24)
    plt.yticks(family = 'Times new roman', fontsize = 24)
    mpl.rcParams['font.sans-serif'] = ['SimHei']
    # plt.xlabel('最优分支反馈迭代次数', fontsize = 24)
    # plt.ylabel('冲突数量（列车）', fontsize = 24)
    plt.xlabel('Number of iterations for the optimal branch', fontsize = 24, family = 'Times new roman')
    plt.ylabel('Number of conflicts', fontsize = 24, family = 'Times new roman')
    plt.xticks([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], family = 'Times new roman')
    plt.yticks([0,1,2,3,4,5,6,7], family = 'Times new roman')
    plt.legend(fontsize = 20)
    plt.show()
