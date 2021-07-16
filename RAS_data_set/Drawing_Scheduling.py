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
MAX_AGENT_NO = 48
g_number_of_stations = 8
DownStream = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
UpStream = [24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47]
MAX_TIME_INTERVAL = 320 * 60
g_number_of_iterations = 20

class Agent:
    def __init__(self):
        self.agent_no = 0
        self.node_list = []
        self.time_list = []

g_agent_list = []
station_list = ['W1-B','W1-1','W1-2','W1-3','W1-4','W1-5','W1-VI','W1-VII','W1-8','W1-A', \
                'W2-B','W2-1','W2-II','W2-III','W2-4','W2-5','W2-6','W2-A', \
                'W3-B','W3-1','W3-II','W3-III','W3-4','W3-5','W3-6','W3-7','W3-A', \
                'W4-B','W4-1','W4-2','W4-3','W4-4','W4-V','W4-VI','W4-7','W4-A', \
                'W5-B','W5-1','W5-2','W5-III','W5-IV','W5-5','W5-6','W5-A', \
                'W6-B','W6-1','W6-2','W6-III','W6-IV','W6-5', 'W6-A', \
                'W7-B','W7-1','W7-II','W7-III','W7-4','W7-5','W7-A', \
                'W8-B','W8-1','W8-II','W8-3','W8-IV','W8-5', 'W8-A']

station_coord = [0,1,2,3,4,5,6,7,8,9,
                 15,16,17,18,19,20,21,22,
                 27,28,29,30,31,32,33,34,35,
                 40,41,42,43,44,45,46,47,48,
                 53,54,55,56,57,58,59,60,
                 65,66,67,68,69,70,71,
                 76,77,78,79,80,81,82,
                 87,88,89,90,91,92,93
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
    os.chdir("./data_set/macro_network/48agents_plot")

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

    # plt.figure(figsize=(210, 297))
    for agent in g_agent_list:
        xlist = agent.time_list
        ylist = agent.node_list
        if(len(xlist) != 0):
            plt.plot(xlist, ylist, color = color_value[agent.agent_no], linewidth = 1.2)
            if agent.agent_no in DownStream:
                plt.text(xlist[-1] - 30, ylist[-1] + 0.2, '%d' % int(agent.agent_no), ha='center', \
                    va= 'bottom', color = color_value[agent.agent_no], weight = 'bold', \
                        family = 'Times new roman', fontsize= 6)
            if agent.agent_no in UpStream:
                plt.text(xlist[-1] - 30, ylist[-1] - 1.5, '%d' % int(agent.agent_no), ha='center', \
                    va= 'bottom', color = color_value[agent.agent_no], weight = 'bold', \
                        family = 'Times new roman', fontsize= 6)

    plt.grid(True) 
    plt.ylim(0, max(station_coord))

    plt.xlim(0, MAX_TIME_INTERVAL)
    plt.xticks([600 * i for i in range(int(MAX_TIME_INTERVAL/600))] , family = 'Times new roman', fontsize = 7)
 

    plt.yticks(station_coord, station_list, family = 'Times new roman', fontsize = 5)
    from pylab import mpl
    mpl.rcParams['font.sans-serif'] = ['SimHei']
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
        if (one_row[0]=='W1'):
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
