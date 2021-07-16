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

# Input Paramter
MAX_AGENT_NO = 24
DownStream = [0,1,2]
UpStream = [3,4,5]
MAX_TIME_INTERVAL = 180 * 60

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
        19:'cornsilk',
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
    }



g_iteration_list = []
g_upper_bound_list = []
g_lower_bound_list = []
g_unfeasible_agent_list = []


if __name__ == '__main__':
    os.chdir("./data_set/macro_network")

    # Step 1: Drawing Timetable 
    file = 'output_drawing_agent_file_compare.csv'
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

    # Rescheduling
    for agent in g_agent_list[12:24]:
        xlist = agent.time_list
        ylist = agent.node_list
        if(len(xlist) != 0):
            plt.plot(xlist, ylist, color = color_value[agent.agent_no - 12], linewidth = 1, linestyle = '--')
            # plt.text(xlist[0] - 30, ylist[0] + 0.2, '%d’' % int(agent.agent_no - 12), ha='center', va= 'bottom', color = color_value[agent.agent_no - 12], weight = 'bold', family = 'Times new roman', fontsize= 10)

    # Origin secheduling 
    for agent in g_agent_list[0:12]:
        xlist = agent.time_list
        ylist = agent.node_list
        if(len(xlist) != 0):
            plt.plot(xlist, ylist, color = color_value[agent.agent_no], linewidth = 1.1)
            plt.text(xlist[-1] - 30, ylist[-1] + 0.2, '%d' % int(agent.agent_no), ha='center', va= 'bottom', color = color_value[agent.agent_no], weight = 'bold', family = 'Times new roman', fontsize= 10)


    plt.grid(True) 
    plt.ylim(0, max(station_coord))

    plt.xlim(0, MAX_TIME_INTERVAL)
    plt.xticks([0,300,600,900,1200,1500,1800,2100,2400,2700,3000,\
                3300,3600,3900,4200,4500,4800,5100,5400,5700,6000,\
                6300,6600,6900,7200,7500,7800,8100,8400,8700,9000,\
                9300,9600,9900,10200,10500,10800], family = 'Times new roman', fontsize = 7.5)
 

    plt.yticks(station_coord, station_list, family = 'Times new roman', fontsize = 5)
    from pylab import mpl
    mpl.rcParams['font.sans-serif'] = ['SimHei']
    plt.xlabel('时间（秒）', fontsize = 12)
    plt.ylabel('车站/股道', fontsize = 12)
    plt.show()


    # Step 2: Drawing convergence curve
    g_iteration_list = []
    g_upper_bound_list = []
    g_lower_bound_list = []
    g_unfeasible_agent_list = []
    
    plt.clf
    file = 'bound.csv'
    csv_file = open(file,'rU') 
    csv_reader = csv.reader(csv_file)

    for one_line in csv_reader:
        if (one_line[0]=='iteration'):
            continue
        g_iteration_list.append(int(one_line[0]) + 1)
        g_upper_bound_list.append(int(one_line[2]))
        g_lower_bound_list.append(float(one_line[1]))
        g_unfeasible_agent_list.append(int(one_line[3]))
    
    plt.plot(g_iteration_list, g_unfeasible_agent_list, color = color_value[0], linewidth = 1.2, label='Unfeasible agent')
    plt.scatter(g_iteration_list, g_unfeasible_agent_list, color = color_value[0], linewidth = 1.2)
    

    plt.xticks(family = 'Times new roman', fontsize = 24)
    plt.yticks(family = 'Times new roman', fontsize = 24)
    mpl.rcParams['font.sans-serif'] = ['SimHei']
    plt.xlabel('反馈迭代次数', fontsize = 24)
    plt.ylabel('冲突数量（列车·站）', fontsize = 24)
    plt.xticks([1,2,3,4,5,6,7,8,9,10], family = 'Times new roman')
    plt.yticks([0,1,2,3,4,5,6,7], family = 'Times new roman')
    # plt.show()
