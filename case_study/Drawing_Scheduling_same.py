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
MAX_AGENT_NO = 7
DownStream = [0, 1, 2, 4, 5, 6]
UpStream = [3]
MAX_TIME_INTERVAL = 5100
MAX_STATION_NO = 21

class Agent:
    def __init__(self):
        self.agent_no = 0
        self.node_list = []
        self.time_list = []

g_agent_list = []
# station_list = ['W6-B','W6-1','W6-2','W6-III','W6-IV','W6-5', 'W6-A', \
#                 'W7-B','W7-1','W7-II','W7-III','W7-4','W7-A', \
#                 'W8-B','W8-1','W8-II','W8-3','W8-IV','W8-5', 'W8-A']
station_list = ['W6-B','W6-1','W6-2','W6-III','W6-IV','W6-5', 'W6-A', \
                'W7-B','W7-1','W7-II','W7-III','W7-4','W7-A', \
                'W8-B','W8-1','W8-II','W8-3','W8-IV','W8-5', 'W8-A']

station_coord = [0,1,2,3,4,5,6,
                 11,12,13,14,15,16,
                 24,25,26,27,28,29,30]
station_map = {}

color_value = {
        0: 'midnightblue', 
        1: 'mediumblue', 
        2:'c',
        3:'orangered',
        4:'m',
        5:'fuchsia',
        6:'olive',
        7:'aliceblue',
        8:'palegreen',
        9:'aquamarine',
        10:'azure',
        11:'beige',
        12:'tan',
        13:'black',
        14:'purple',
        15:'blue',
        16:'blueviolet',
        17:'brown',
        18:'burlywood',
        19:'cadetblue',
        20:'dimgray',
        21:'chocolate',
        22:'coral',
        23:'cornflowerblue',
        24:'cornsilk',
        25:'crimson',
        26:'cyan',
        27:'darkblue',
        28:'darkcyan',
        29:'darkgoldenrod',
        30:'darkgray',
        31:'darkgreen',
        32:'darkkhaki',
        33:'darkmagenta',
        34:'darkolivegreen',
        35:'darkorange',
        36:'darkorchid',
        37:'white'
    }



g_iteration_list = []
g_upper_bound_list = []
g_lower_bound_list = []
g_unfeasible_agent_list = []


if __name__ == '__main__':
    os.chdir("./data_set/output_branch") # for detailed branch and bound
    # os.chdir("./data_set/output") # for branching nodes

    # Step 1: Drawing Timetable 
    # file = 'tree_node_6_seq_(0, 4, 3, 1, 2, 5)_conflict_1.csv'
    file = 'feasible_tree_node_6_seq_(0, 4, 3, 1, 2, 5)_conflict_0.csv'

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


    for agent in g_agent_list[0:6]:
        xlist = agent.time_list
        ylist = agent.node_list
        if(len(xlist) != 0):
            plt.plot(xlist, ylist, color = color_value[agent.agent_no], linewidth = 1.2)
            plt.text(xlist[-1] - 30, ylist[-1] + 0.2, '%d' % int(agent.agent_no), ha='center', va= 'bottom', color = color_value[agent.agent_no], weight = 'bold', family = 'Times new roman', fontsize= 10)

    xlist = g_agent_list[6].time_list
    ylist = g_agent_list[6].node_list
    if(len(xlist) != 0):
        plt.plot(xlist, ylist, color = color_value[2], linewidth = 1.2, linestyle = '--')
        plt.text(xlist[-1] - 30, ylist[-1] + 0.2, '2’', ha='center', va= 'bottom', color = color_value[2], weight = 'bold', family = 'Times new roman', fontsize= 10)



    plt.grid(True) 
    plt.ylim(0, max(station_coord))

    plt.xlim(0, MAX_TIME_INTERVAL)
    plt.xticks([0,300,600,900,1200,1500,1800,2100,2400,2700,3000,3300,3600,3900,4200,4500,4800,5100], family = 'Times new roman')
 

    plt.yticks(station_coord, station_list, family = 'Times new roman')
    from pylab import mpl
    mpl.rcParams['font.sans-serif'] = ['Times new roman']
    plt.xlabel('Time (Second)', fontsize = 12)
    plt.ylabel('Station/Track', fontsize = 12)
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
    plt.show()



