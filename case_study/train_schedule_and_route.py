'''
    Train scheduling problem for multi-level railway network
    @ Portions Copyright 2021 Entai Wang
    @ State Key Lab of Rail Traffic Control & Safety Beijing Jiaotong University 
'''

import os
import matplotlib.pyplot as plt
from random import randint
import numpy as np
import csv
import time
from queue import Queue,LifoQueue,PriorityQueue
import itertools
import copy
import sys

g_station_list = [6,8,7]
g_agent_map_dict = {}
g_agent_map = []
g_workbench_list = []
g_outer_iteration = 1

g_number_of_nodes = 0
g_number_of_links = 0
g_number_of_agents = 0
MAX_LABEL_COST = 99999

g_node_list = []
g_link_list = []
g_agent_list = []
g_upper_bound_list = []
g_lower_bound_list = []
effective_agent = []

g_internal_node_seq_no_dict = {}
g_external_node_id_dict = {}
g_internal_link_seq_no_dict = {}
g_external_link_id_dict = {}
g_link_key_to_seq_no_dict = {}
g_x_axis_unit = 1  # in minutes, default unit value of x axis is 1 min

g_best_upper_bound = 99999
g_best_lower_bound = -99999
optimality_gap = 0
freStop_best_feas_Upper_bound_flag = 1

g_outer_upper_bound_list = []
g_outer_lower_bound_list = []
g_outer_unfeasiable_list = []
g_agent_sequence = []

BB_tree_queue = Queue(maxsize = 0)

def g_reset_global_parameter():
    global g_number_of_nodes
    global g_number_of_links
    global g_number_of_agents
    global MAX_LABEL_COST

    global g_node_list
    global g_link_list
    global g_agent_list
    global g_upper_bound_list
    global g_lower_bound_list

    global g_internal_node_seq_no_dict
    global g_external_node_id_dict
    global g_internal_link_seq_no_dict
    global g_external_link_id_dict
    global g_link_key_to_seq_no_dict
    global g_x_axis_unit

    global g_best_upper_bound
    global g_best_lower_bound
    global optimality_gap
    global freStop_best_feas_Upper_bound_flag

    g_number_of_nodes = 0
    g_number_of_links = 0
    g_number_of_agents = 0
    MAX_LABEL_COST = 99999

    g_node_list = []
    g_link_list = []
    g_agent_list = []
    g_upper_bound_list = []
    g_lower_bound_list = []

    g_internal_node_seq_no_dict = {}
    g_external_node_id_dict = {}
    g_internal_link_seq_no_dict = {}
    g_external_link_id_dict = {}
    g_link_key_to_seq_no_dict = {}
    g_x_axis_unit = 1  # in minutes, default unit value of x axis is 1 min

    g_best_upper_bound = 99999
    g_best_lower_bound = -99999
    optimality_gap = 0
    freStop_best_feas_Upper_bound_flag = 1

class TreeNode:
    def __init__(self):
        self.tree_node_no = None
        self.BB_matrix = []
        self.micro_sequence = []
        self.agent_headway_list = []
        self.terminal_time = 9999
        self.total_cost = 9999

class Node:
    def __init__(self):
        self.name = ''
        self.node_id = 0
        self.node_seq_no = 0
        self.x_corrd = 0
        self.y_coord = 0
        self.display_coord = 0
        self.station_id = 0
        self.station_sequence = 0
        self.outgoing_link_list = [] 

class Link:
    def __init__(self):
        self.name = ''
        self.link_id = 0
        self.from_node_id = 0
        self.to_node_id = 0
        self.direction = 0        
        self.length = 0
        self.lanes = 0
        self.free_speed = 0
        self.capacity = 0
        self.link_type = 0
        self.same_link_id = 0
        self.oppo_link_id = 0
        self.local_y = 0


        self.link_seq_no = 0
        self.from_node_seq_no = 0
        self.to_node_seq_no = 0

        self.left_corrd = 0
        self.right_corrd = 0

        self.time_dependent_departure_LR_multiplier_matrix = []
        self.time_dependent_arrival_LR_multiplier_matrix = []

        self.g_link_time_departure_visit_counts = []
        self.g_link_time_arrival_visit_counts = []
        self.g_link_time_departure_train_visit_flag = []
        self.g_link_time_arrival_train_visit_flag = []

        self.g_link_time_departure_visit_counts_in_upper_bound = []
        self.g_link_time_arrival_visit_counts_in_upper_bound = []
        self.g_link_time_departure_train_visit_flag_in_upper_bound = []
        self.g_link_time_arrival_train_visit_flag_in_upper_bound = []

        self.departure_dependent_travel_time_matrix = []
        self.arrival_dependent_travel_time_matrix = []
        self.time_depedent_capacity_matrix = []

class Agent:
    def __init__(self):
        self.agent_id = 0
        self.o_node_id = 0
        self.d_node_id = 0
        self.o_node_seq_no = 0
        self.d_node_seq_no = 0

        self.earliest_departure_time = 0
        self.latest_arrival_time = 0
        self.frequency = 0
        self.direction = 0
        self.speed_grade = 0

        self.set_of_allowed_links = []
        self.min_link_travel_time = []
        self.max_link_travel_time = []
        self.headway = []

        self.node_sequence = []
        self.time_sequence = []  # in minutes
        self.xlist = []  # the time coordinate
        self.ylist = []  # the space coordinate
        self.agent_type = ''

        self.path_new_link_id_vector = []
        self.path_link_TA_vector = []
        self.path_link_TD_vector = []

        self.path_new_node_id_vector = []
        self.path_new_timestamp_vector = []

        self.partial_schedule_node_vector = []
        self.partial_schedule_node_TD = []
        self.partial_schedule_node_TA = []

        self.path_link_seq_no_vector = []
        self.path_timestamp_vector = []
        self.path_node_id_vector = []

        self.free_flow_travel_time = 0
        self.travel_time_in_dual_solution = 0

        self.path_link_seq_no_vector_upper_bound = []
        self.path_timestamp_vector_upper_bound = []
        self.path_node_id_vector_upper_bound = []
        self.path_new_link_id_vector_upper_bound = []
        self.path_link_TA_vector_upper_bound = []
        self.path_link_TD_vector_upper_bound = []
        self.path_new_node_id_vector_upper_bound = []
        self.path_new_timestamp_vector_upper_bound = []

        # Output solution
        self.g_output_link_NO_vector = []
        self.g_output_link_TD_vector = []
        self.g_output_link_TA_vector = []

class MapAgent:
    def __init__(self):
        station_name = ''
        type_name = ''
        macro_to_node_id = None
        macro_to_node_id = None
        micro_from_node_id = None
        micro_to_node_id = None
        set_of_allowed_links = []
        min_link_travel_time = []
        max_link_travel_time = []

class Workbench:
    def __init__(self):
        self.name = ''
        g_node_list = []
        g_link_list = []
        g_agent_list = []

        g_upper_bound_list = []
        g_lower_bound_list = []

        g_internal_node_seq_no_dict = {}
        g_external_node_id_dict = {}
        g_internal_link_seq_no_dict = {}
        g_external_link_id_dict = {}
        g_link_key_to_seq_no_dict = {}
        g_x_axis_unit = 1  # in minutes, default unit value of x axis is 1 min

        g_best_upper_bound = 99999
        g_best_lower_bound = -99999
        optimality_gap = 0
        freStop_best_feas_Upper_bound_flag = 1
        effective_agent = []
        g_agent_sequence = []
        g_agent_sequence_micro = []   


def g_ReadInputData():
    global g_number_of_nodes
    global g_number_of_links
    global g_number_of_agents
    global g_number_of_simulation_intervals
    
    with open('input_node.csv', 'r') as fp:
        internal_node_seq_no = 0
        lines = fp.readlines()
        temp = lines[0].strip().split(',')
        index_name = temp.index('name')
        index_node_id = temp.index('node_id')
        index_zone_id = temp.index('zone_id')
        index_x_corrd = temp.index('x_coord')
        index_y_coord = temp.index('y_coord')
        index_station_seq = temp.index('station_sequence')
        for l in lines[1:]:
            l = l.strip().split(',')
            try:
                node = Node()
                node.name = l[index_name]
                node.node_id = int(l[index_node_id])
                node.zone_id = int(l[index_zone_id])

                # from id to seq no
                g_internal_node_seq_no_dict[node.node_id] = internal_node_seq_no
                # from seq no to id
                g_external_node_id_dict[internal_node_seq_no] = node.node_id

                node.node_seq_no = internal_node_seq_no
                internal_node_seq_no += 1
                node.x_corrd = float(l[index_x_corrd])
                node.y_coord = float(l[index_y_coord])
                # node.station_sequence = int(l[index_station_seq])
                g_node_list.append(node)
                g_number_of_nodes += 1
                # if g_number_of_nodes % 10 == 0:
                    # print('reading {} nodes'.format(g_number_of_nodes))
            except:
                print('Fail to read the input node file.')
        # print('number of nodes:{}'.format(g_number_of_nodes))

    with open('input_link.csv', 'r') as fp:
        internal_link_seq_no = 0
        lines = fp.readlines()
        temp = lines[0].strip().split(',')
        index_name = temp.index('name')
        index_link_id = temp.index('link_id')
        index_from_node_id = temp.index('from_node_id')
        index_to_node_id = temp.index('to_node_id')
        index_direction_id = temp.index('direction')
        index_length = temp.index('length')
        index_lanes = temp.index('lanes')
        index_free_speed = temp.index('free_speed')
        index_capacity = temp.index('capacity')
        index_link_type = temp.index('link_type')
        index_same_link_id = temp.index('same_link_id')
        index_oppo_link_id = temp.index('oppo_link_id')
        index_local_y = temp.index('local_y')
        # index_display_seq = temp.index('display_sequence')

        for l in lines[1:]:
            l = l.strip().split(',')
            # try:
            link = Link()
            link.name = l[index_name]
            link.link_id = int(l[index_link_id])

            # from id to seq no
            g_internal_link_seq_no_dict[link.link_id] = internal_link_seq_no
            # from seq no to id
            g_external_link_id_dict[internal_link_seq_no] = link.link_id
            
            link.from_node_id = int(l[index_from_node_id])
            link.to_node_id = int(l[index_to_node_id])
            link.from_node_seq_no = g_internal_node_seq_no_dict[link.from_node_id]
            link.to_node_seq_no = g_internal_node_seq_no_dict[link.to_node_id]
            link.link_seq_no = internal_link_seq_no
            internal_link_seq_no += 1
            
            link.direction = int(l[index_direction_id])
            link.length = float(l[index_length])
            link.lanes = int(l[index_lanes])
            link.free_speed = float(l[index_free_speed])

            link.capacity = int(l[index_capacity])
            link.link_type = int(l[index_link_type]) 
            
            link.same_link_id = int(l[index_same_link_id])
            link.oppo_link_id = int(l[index_oppo_link_id])
            link.local_y = int(l[index_local_y])
            link.departure_dependent_travel_time_matrix = [0] * g_number_of_simulation_intervals
            link.arrival_dependent_travel_time_matrix = [0] * g_number_of_simulation_intervals


            g_node_list[link.from_node_seq_no].outgoing_link_list.append(link.link_seq_no)

            link_key = link.from_node_seq_no * 100000 + link.to_node_seq_no
            g_link_key_to_seq_no_dict[link_key] = link.link_seq_no
            g_link_list.append(link)
            g_number_of_links += 1
            # if g_number_of_links % 10 == 0:
                # print('reading {} links'.format(g_number_of_links))
            # except:
            #     print('Fail to read the input road link file.')
        # print('number of road links:{}'.format(g_number_of_links))  

    with open('input_agent.csv', 'r') as fp:
        lines = fp.readlines()
        temp = lines[0].strip().split(',')
        index_agent_id = temp.index('agent_id')
        index_o_node_id = temp.index('from_origin_node_id')
        index_d_node_id = temp.index('to_destination_node_id')

        index_earliest_departure_time =temp.index('earliest_departure_time')
        try:
            index_latest_arrival_time = temp.index('latest_arrival_time')
        except:
            temp_value = 0
        index_frequency = temp.index('frequency')
        index_direction = temp.index('direction')
        index_speed_grade = temp.index('speed_grade')
        index_set_of_allowed_links = temp.index('set_of_allowed_links')
        index_min_link_travel_time = temp.index('min_link_travel_time')
        index_max_link_travel_time = temp.index('max_link_travel_time')
        try:
            index_headway = temp.index('headway')
        except:
            temp_value = 0

        for l in lines[1:]:
            l = l.strip().split(',')
            try:
                agent = Agent()
                agent.agent_id = int(l[index_agent_id])
                agent.o_node_id = int(l[index_o_node_id])
                agent.d_node_id = int(l[index_d_node_id])
                agent.o_node_seq_no = g_internal_node_seq_no_dict[agent.o_node_id]
                agent.d_node_seq_no = g_internal_node_seq_no_dict[agent.d_node_id]

                agent.earliest_departure_time = int(l[index_earliest_departure_time])
                try:
                    agent.latest_arrival_time = int(l[index_latest_arrival_time])
                except:
                    agent.latest_arrival_time = 0
                agent.frequency = int(l[index_frequency])
                agent.direction = int(l[index_direction])
                agent.speed_grade = int(l[index_speed_grade])

                allowed_links_list = l[index_set_of_allowed_links].strip().split(';')
                for k in range(0, len(allowed_links_list)):
                    if allowed_links_list[k] != '':
                        agent.set_of_allowed_links.append(g_internal_link_seq_no_dict[int(allowed_links_list[k])])
                min_link_travel_time_list = l[index_min_link_travel_time].strip().split(';')
                for k in range(0, len(min_link_travel_time_list)):
                    if min_link_travel_time_list[k] != '':
                        agent.min_link_travel_time.append(int(min_link_travel_time_list[k]))
                max_link_travel_time_list = l[index_max_link_travel_time].strip().split(';')
                for k in range(0, len(max_link_travel_time_list)):
                    if max_link_travel_time_list[k] != '':
                        agent.max_link_travel_time.append(int(max_link_travel_time_list[k]))
                headway_list = l[index_headway].strip().split(';')
                try:
                    for k in range(0, len(headway_list)):
                        if headway_list[k] != '':
                            agent.headway.append(int(headway_list[k]))
                except:
                    index_temp = 0

                g_agent_list.append(agent)
                g_number_of_agents += 1
                # if g_number_of_agents % 10 == 0:
                    # print('reading {} agents'.format(g_number_of_agents))
            except:
                print('Fail to read the input agent file.')
        # print('number of agents:{}'.format(g_number_of_agents))  

def g_agent_list_clear():
    for agent in g_agent_list:
        agent.path_new_link_id_vector.clear()
        agent.path_link_TA_vector.clear()
        agent.path_link_TD_vector.clear()
        agent.path_new_node_id_vector.clear()
        agent.path_new_timestamp_vector.clear()
        agent.partial_schedule_node_vector.clear()
        agent.partial_schedule_node_TD.clear()
        agent.partial_schedule_node_TA.clear()

        agent.path_new_link_id_vector.clear()
        agent.path_link_TA_vector.clear()
        agent.path_link_TD_vector.clear()

        agent.path_new_node_id_vector.clear()
        agent.path_new_timestamp_vector.clear()

        agent.partial_schedule_node_vector.clear()
        agent.partial_schedule_node_TD.clear()
        agent.partial_schedule_node_TA.clear()

        agent.path_link_seq_no_vector.clear()
        agent.path_timestamp_vector.clear()
        agent.path_node_id_vector.clear()

        agent.free_flow_travel_time = 0
        agent.travel_time_in_dual_solution = 0

        agent.path_link_seq_no_vector_upper_bound.clear()
        agent.path_timestamp_vector_upper_bound.clear()
        agent.path_node_id_vector_upper_bound.clear()
        agent.path_new_link_id_vector_upper_bound.clear()
        agent.path_link_TA_vector_upper_bound.clear()
        agent.path_link_TD_vector_upper_bound.clear()
        agent.path_new_node_id_vector_upper_bound.clear()
        agent.path_new_timestamp_vector_upper_bound.clear()

def g_link_list_clear():
    for link in g_link_list:
        link.departure_dependent_travel_time_matrix = [0] * g_number_of_simulation_intervals
        link.arrival_dependent_travel_time_matrix = [0] * g_number_of_simulation_intervals

# Dynamic programming for LR lower bound
def optimal_ST_dynamic_programming_LR(arg, departure_time_beginning, arrival_time_ending, LR_iteration):

    # Label correcting
    agent_no = arg[0]
    internal_origin_node_seq_no = arg[1]
    departure_time_beginning = arg[2]
    arrival_time_ending = arg[3]
    m_origin_node = internal_origin_node_seq_no
    m_departure_time_beginning = departure_time_beginning 
    m_arrival_time_ending = arrival_time_ending


    if(m_origin_node < 0):
        return -1
    
    total_cost = MAX_LABEL_COST

    if(len(g_node_list[m_origin_node].outgoing_link_list) == 0):
        return MAX_LABEL_COST

    # Initialization for all space timem_label_cost node
    m_label_cost = [[MAX_LABEL_COST for i in range(g_number_of_simulation_intervals)] for i in range(g_number_of_nodes)]
    m_node_predecessor = [[-1 for i in range(g_number_of_simulation_intervals)] for i in range(g_number_of_nodes)]
    m_time_predecessor = [[-1 for i in range(g_number_of_simulation_intervals)] for i in range(g_number_of_nodes)]

    if(g_agent_list[agent_no].agent_type == 'micro agent'):
        m_label_cost[m_origin_node][departure_time_beginning] = 0
    else:
        for t in range(departure_time_beginning, arrival_time_ending):
            m_label_cost[m_origin_node][t] = 0
    
    for t in range(departure_time_beginning, arrival_time_ending):
        for n in range(0,len(g_node_list)):
            temp_node_no = g_node_list[n].node_seq_no
            
            for link in g_node_list[temp_node_no].outgoing_link_list:
                # Check the link exist in the set of allowed links
                if link not in g_agent_list[agent_no].set_of_allowed_links:
                    continue
                
                from_node = g_link_list[link].from_node_seq_no
                to_node = g_link_list[link].to_node_seq_no
            
                for travel_time in range(g_agent_list[agent_no].min_link_travel_time[link], g_agent_list[agent_no].max_link_travel_time[link] + 1):
                    if(m_label_cost[from_node][t] < MAX_LABEL_COST - 1):
                        travel_cost = travel_time
                        new_to_node_arrival_time = min(t + travel_time, g_number_of_simulation_intervals - 1) 
                        # Plus the waitting time on this link

                        temporary_label_cost = 0
                        sum_of_multipliers = 0

                        n_position = 0
                        TA_left_time = t
                        TD_right_time = new_to_node_arrival_time
                        TA_left_time_headway = 0
                        TD_right_time_headway = 0
                        next_link_no = 0
                        former_link_no = 0

                        n_position = g_agent_list[agent_no].set_of_allowed_links.index(link)

                        # if (g_link_list[link].link_type != 12):
                        if ((g_link_list[link].link_type != 11) & (g_link_list[link].link_type != 12)):
                            # "11" means "siding track" type and "12" means "dummy track" type
                            # Compute left_time_headway and right_time_headway
                            if(n_position == 0): # First section
                                TA_left_time_headway = g_departure_headway_stop # departure event
                                next_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position + 1]
                                if (g_link_list[next_link_no].link_type == 11): # next->arrival event
                                    # TD_right_time_headway = g_arrival_headway_stop
                                    TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                                else:
                                    TD_right_time_headway = g_arrival_headway_passing

                            elif(n_position == len(g_agent_list[agent_no].set_of_allowed_links) - 2): # Last section
                                # the last one is dummuy link, not the destination link
                                TD_right_time_headway = g_arrival_headway_stop
                                former_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position - 1]
                                if(g_link_list[former_link_no].link_type == 11): # former->departure event
                                    # TA_left_time_headway = g_departure_headway_stop
                                    TA_left_time_headway = g_agent_list[agent_no].headway[former_link_no]
                                else: # passing event
                                    TA_left_time_headway = g_departure_headway_passing
                            else: # intermediate section
                                next_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position + 1]
                                if(g_link_list[next_link_no].link_type == 11): # next->arrival event
                                    # TD_right_time_headway = g_arrival_headway_stop
                                    TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                # elif (g_link_list[next_link_no].link_type == 13):
                                    # TD_right_time_headway = g_arrival_headway_stop_DF
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                else: # passing event
                                    TD_right_time_headway = g_arrival_headway_passing
                                
                                former_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position - 1]
                                if(g_link_list[former_link_no].link_type == 11): # former->departure event
                                    # TA_left_time_headway = g_departure_headway_stop
                                    TA_left_time_headway = g_agent_list[agent_no].headway[former_link_no]
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                # elif (g_link_list[next_link_no].link_type == 13):
                                    # TD_right_time_headway = g_arrival_headway_stop_DF
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                else: # passing event
                                    TA_left_time_headway = g_departure_headway_passing
                            
                            # LR multipliers in TA_left side
                            for time in range(TA_left_time, min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                                time_temp = min(time, g_number_of_simulation_intervals)
                                sum_of_multipliers = sum_of_multipliers + g_link_list[link].time_dependent_departure_LR_multiplier_matrix[time_temp]

                            # LR multipliers in TD_right_side
                            for time in range(TD_right_time, min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                                time_temp = min(time, g_number_of_simulation_intervals)
                                sum_of_multipliers = sum_of_multipliers + g_link_list[link].time_dependent_arrival_LR_multiplier_matrix[time_temp]

                        temporary_label_cost = m_label_cost[from_node][t] + travel_cost + sum_of_multipliers

                        if (temporary_label_cost < m_label_cost[to_node][new_to_node_arrival_time]):
                            # Update label cost and predecessor
                            m_label_cost[to_node][new_to_node_arrival_time] = temporary_label_cost
                            m_node_predecessor[to_node][new_to_node_arrival_time] = from_node
                            m_time_predecessor[to_node][new_to_node_arrival_time] = t

    return total_cost, m_label_cost, m_node_predecessor, m_time_predecessor

# Backtrace for LR lower bound
def find_ST_path_for_agents_LR(arg, LR_iteration):

    total_cost = MAX_LABEL_COST
    agent_no = arg[0]
    internal_origin_node_seq_no = arg[1]
    departure_time_beginning = arg[2]
    arrival_time_ending = arg[3]
    m_origin_node = internal_origin_node_seq_no
    m_departure_time_beginning = departure_time_beginning
    m_arrival_time_ending = arrival_time_ending
    
    reverse_path_node_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    reverse_path_time_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    reverse_path_cost_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS

    path_node_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    path_link_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    path_time_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    path_cost_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS

        
    # Performing one to all Space time shortest path
    return_value, m_label_cost, m_node_predecessor, m_time_predecessor = \
         optimal_ST_dynamic_programming_LR(arg, m_departure_time_beginning, m_arrival_time_ending, LR_iteration)

    total_cost = MAX_LABEL_COST

    # Backtrace in the shortest path tree
    p_agent = None
    p_agent = g_agent_list[agent_no]
    p_agent.path_link_seq_no_vector.clear()
    p_agent.path_timestamp_vector.clear()

    if(return_value == -1):
        print('agent ' + p_agent.agent_id + ' can not find the ST path')
        return

    destination_node_seq_no = g_internal_node_seq_no_dict[p_agent.d_node_id]
    min_cost_time_index = m_arrival_time_ending

    total_cost = m_label_cost[destination_node_seq_no][min_cost_time_index]

    for t in range(m_arrival_time_ending, m_departure_time_beginning, -1):
        if(m_label_cost[destination_node_seq_no][t] <= total_cost):
            min_cost_time_index = t
            total_cost = m_label_cost[destination_node_seq_no][t]

    # Initialize the reversed array
    node_size = 0
    reverse_path_node_seq[node_size] = destination_node_seq_no
    reverse_path_time_seq[node_size] = min_cost_time_index
    reverse_path_cost_seq[node_size] = m_label_cost[destination_node_seq_no][min_cost_time_index]

    node_size = node_size + 1

    pred_node = m_node_predecessor[destination_node_seq_no][min_cost_time_index]
    pred_time = m_time_predecessor[destination_node_seq_no][min_cost_time_index]

    while((pred_node != -1) & (node_size < MAX_NUMBER_OF_TIME_INTERVALS)):
        # Scanning backward in the predecessor array
        reverse_path_node_seq[node_size] = pred_node
        reverse_path_time_seq[node_size] = pred_time
        reverse_path_cost_seq[node_size] = m_label_cost[pred_node][pred_time]
        
        node_size = node_size + 1
        
        pred_node_record = pred_node
        pred_time_record = pred_time
        
        pred_node = m_node_predecessor[pred_node_record][pred_time_record]
        pred_time = m_time_predecessor[pred_node_record][pred_time_record]
    
    # Reverse the sequence
    path_node_seq = list(filter(lambda x:x!=-1,reverse_path_node_seq))
    path_node_seq.reverse()
    path_time_seq = list(filter(lambda x:x!=-1,reverse_path_time_seq))
    path_time_seq.reverse()
    path_cost_seq = list(filter(lambda x:x!=-1,reverse_path_cost_seq))
    path_cost_seq.reverse()

    # Put the node seq and link seq into agent 
    p_agent.path_node_id_vector.clear()
    for i in range(0, node_size):
        p_agent.path_node_id_vector.append(path_node_seq[i])
        p_agent.path_timestamp_vector.append(path_time_seq[i])

    for i in range(0, node_size-1):
        for link in g_link_list:
            if ((path_node_seq[i] == link.from_node_seq_no) & (path_node_seq[i+1] == link.to_node_seq_no)):
                 p_agent.path_link_seq_no_vector.append(link.link_seq_no)
                 break
    
    travel_time_return_value = path_time_seq[node_size - 1] - path_time_seq[0] # Running time
    path_number_of_nodes = node_size

    # Distinguish the arrival time and departure time for each node
    temp_node_id = p_agent.path_node_id_vector[0]
    node_arrival_time = p_agent.path_timestamp_vector[0]
    node_departure_time = p_agent.path_timestamp_vector[0]

    for n in range(0, len(p_agent.path_node_id_vector)-1):
        if (node_departure_time == p_agent.path_timestamp_vector[n+1]): # Waitting at here
            node_departure_time = p_agent.path_timestamp_vector[n+1]
        else: #Going to next station
            p_agent.path_new_node_id_vector.append(temp_node_id)
            p_agent.path_new_timestamp_vector.append(node_arrival_time)
            p_agent.path_new_timestamp_vector.append(node_departure_time)

            temp_node_id = p_agent.path_node_id_vector[n+1]
            node_arrival_time = p_agent.path_timestamp_vector[n+1]
            node_departure_time = p_agent.path_timestamp_vector[n+1]
        
    p_agent.path_new_node_id_vector.append(temp_node_id)
    p_agent.path_new_timestamp_vector.append(node_arrival_time)
    p_agent.path_new_timestamp_vector.append(node_departure_time)

    # Get the path (link) vector, TA and TD vector
    
    temp_number = 1
    for n in range(0, len(p_agent.path_new_node_id_vector)-1):
        p_agent.path_link_TA_vector.append(p_agent.path_new_timestamp_vector[temp_number])
        temp_number = temp_number + 2

    temp_number = 3
    for n in range(1, len(p_agent.path_new_node_id_vector)):
        p_agent.path_link_TD_vector.append(p_agent.path_new_timestamp_vector[temp_number])
        temp_number = temp_number + 2

    for n in range(0, len(p_agent.path_new_node_id_vector)-1):
        from_node_no = list(g_internal_node_seq_no_dict.keys())[list(g_internal_node_seq_no_dict.values()).index(p_agent.path_new_node_id_vector[n])]
        to_node_no = list(g_internal_node_seq_no_dict.keys())[list(g_internal_node_seq_no_dict.values()).index(p_agent.path_new_node_id_vector[n+1])]

        for link in g_link_list:
            if ((from_node_no == link.from_node_id) & (to_node_no == link.to_node_id)):
                temp_link_no = link.link_seq_no
                p_agent.path_new_link_id_vector.append(temp_link_no)

    n = len(p_agent.path_link_TD_vector)
    if(LR_iteration == 0):
        p_agent.free_flow_travel_time = p_agent.path_link_TD_vector[n-1] - p_agent.path_link_TD_vector[0]
        p_agent.travel_time_in_dual_solution = p_agent.path_link_TD_vector[n-1] - p_agent.path_link_TD_vector[0]
    else:
        p_agent.travel_time_in_dual_solution = p_agent.path_link_TD_vector[n-1] - p_agent.path_link_TD_vector[0]

    g_agent_list[agent_no] = p_agent
    return total_cost

# Dynamic programming for LR upper bound
def optimal_ST_dynamic_programming(arg, LR_iteration):
    # Label correcting
    agent_no = arg[0]
    internal_origin_node_seq_no = arg[1]
    departure_time_beginning = arg[2]
    arrival_time_ending = arg[3]
    m_origin_node = internal_origin_node_seq_no
    m_departure_time_beginning = departure_time_beginning 
    m_arrival_time_ending = arrival_time_ending

    if(m_origin_node < 0):
        return -1
    
    total_cost = MAX_LABEL_COST

    if(len(g_node_list[m_origin_node].outgoing_link_list) == 0):
        return MAX_LABEL_COST

    # Initialization for all space timem_label_cost node
    m_label_cost = [[MAX_LABEL_COST for i in range(g_number_of_simulation_intervals)] for i in range(g_number_of_nodes)]
    m_node_predecessor = [[-1 for i in range(g_number_of_simulation_intervals)] for i in range(g_number_of_nodes)]
    m_time_predecessor = [[-1 for i in range(g_number_of_simulation_intervals)] for i in range(g_number_of_nodes)]

    if(g_agent_list[agent_no].agent_type == 'micro agent'):
        m_label_cost[m_origin_node][departure_time_beginning] = 0
    else:
        for t in range(departure_time_beginning, g_number_of_simulation_intervals):
            m_label_cost[m_origin_node][t] = 0
    
    for t in range(departure_time_beginning, arrival_time_ending):
        for n in range(0,len(g_node_list)):
            temp_node_no = g_node_list[n].node_seq_no
            
            for link in g_node_list[temp_node_no].outgoing_link_list:
                # Check the link exist in the set of allowed links
                if link not in g_agent_list[agent_no].set_of_allowed_links:
                    continue
                
                from_node = g_link_list[link].from_node_seq_no
                to_node = g_link_list[link].to_node_seq_no
                
                for travel_time in range(g_agent_list[agent_no].min_link_travel_time[link], g_agent_list[agent_no].max_link_travel_time[link] + 1):
                    if(m_label_cost[from_node][t] < MAX_LABEL_COST - 1):
                        # For feasible space-time arc only
                        travel_cost = travel_time
                        if(t + travel_time > (g_number_of_simulation_intervals - 1)):
                            continue
                        new_to_node_arrival_time = min(t + travel_time, g_number_of_simulation_intervals - 1) 
                        # Plus the waitting time on this link

                        temporary_label_cost = 0
                        sum_of_multipliers = 0

                        n_position = 0
                        TA_left_time = t
                        TD_right_time = new_to_node_arrival_time
                        TA_left_time_headway = 0
                        TD_right_time_headway = 0
                        next_link_no = 0
                        former_link_no = 0
                        isFeasible = 0

                        n_position = g_agent_list[agent_no].set_of_allowed_links.index(link)

                        # if (g_link_list[link].link_type != 12):
                        if ((g_link_list[link].link_type != 11) & (g_link_list[link].link_type != 12)):
                            # "11" means "siding track" type and "12" means "dummy track" type
                            # Compute left_time_headway and right_time_headway
                            if(n_position == 0): # First section
                                TA_left_time_headway = g_departure_headway_stop # departure event
                                next_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position + 1]
                                if (g_link_list[next_link_no].link_type == 11): # next->arrival event
                                    # TD_right_time_headway = g_arrival_headway_stop
                                    TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                                else:
                                    TD_right_time_headway = g_arrival_headway_passing

                            elif(n_position == len(g_agent_list[agent_no].set_of_allowed_links) - 2): # Last section
                                # the last one is dummuy link, not the destination link
                                # TD_right_time_headway = g_arrival_headway_stop
                                TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]

                                former_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position - 1]
                                if(g_link_list[former_link_no].link_type == 11): # former->departure event
                                    # TA_left_time_headway = g_departure_headway_stop
                                    TA_left_time_headway = g_agent_list[agent_no].headway[former_link_no]

                                else: # passing event
                                    TA_left_time_headway = g_departure_headway_passing
                            else: # intermediate section
                                next_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position + 1]
                                if(g_link_list[next_link_no].link_type == 11): # next->arrival event
                                    # TD_right_time_headway = g_arrival_headway_stop
                                    TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                # elif (g_link_list[next_link_no].link_type == 13):
                                #     TD_right_time_headway = g_agent_list[agent_no].max_link_travel_time[next_link_no] + g_arrival_headway_stop
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                else: # passing event
                                    TD_right_time_headway = g_arrival_headway_passing

                                former_link_no = g_agent_list[agent_no].set_of_allowed_links[n_position - 1]
                                if(g_link_list[former_link_no].link_type == 11): # former->departure event
                                    # TA_left_time_headway = g_departure_headway_stop
                                    TA_left_time_headway = g_agent_list[agent_no].headway[former_link_no]
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                # elif (g_link_list[next_link_no].link_type == 13):
                                #     TA_left_time_headway = g_agent_list[agent_no].max_link_travel_time[former_link_no] + g_departure_headway_stop
                                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                                else: # passing event
                                    TA_left_time_headway = g_departure_headway_passing
                        
                        # # # # # # # # # # # # # # # # # # # # # # # # 
                        # Forbidding the headway space time arc.
                        # In the DP process, if an incompatible arc is encountered, skip over
                        if ((g_link_list[link].link_type != 11) & (g_link_list[link].link_type != 12)):
                            # TA
                            for time in range(TA_left_time, min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                                temp_time = min(time, g_number_of_simulation_intervals - 1)
                                if(g_link_list[link].departure_dependent_travel_time_matrix[temp_time] < 0):
                                    isFeasible = 1
                                    travel_cost = MAX_LABEL_COST
                                    break
                                if(g_link_list[link].same_link_id != 1000):
                                    same_link_no = g_internal_link_seq_no_dict[g_link_list[link].same_link_id]
                                    if(g_link_list[same_link_no].departure_dependent_travel_time_matrix[temp_time] < 0):
                                        isFeasible = 1
                                        travel_cost = MAX_LABEL_COST
                                        break 
                            if(isFeasible == 1):
                                break

                            # TD
                            for time in range(TD_right_time, min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                                temp_time = min(time, g_number_of_simulation_intervals - 1)
                                if(g_link_list[link].arrival_dependent_travel_time_matrix[temp_time] < 0):
                                    isFeasible = 1
                                    travel_cost = MAX_LABEL_COST
                                    break
                                if(g_link_list[link].same_link_id != 1000):
                                    same_link_no = g_internal_link_seq_no_dict[g_link_list[link].same_link_id]
                                    if(g_link_list[same_link_no].arrival_dependent_travel_time_matrix[temp_time] < 0):
                                        isFeasible = 1
                                        travel_cost = MAX_LABEL_COST
                                        break 
                            if(isFeasible == 1):
                                break 
                        # # # # # # # # # # # # # # # # # # # #             

                        # Updating Lagrangian multiplier
                        if (isFeasible == 0):
                            # TA
                            for time in range(TA_left_time, min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                                temp_time = min(time, g_number_of_simulation_intervals - 1)
                                sum_of_multipliers = sum_of_multipliers + g_link_list[link].time_dependent_departure_LR_multiplier_matrix[temp_time]       
                            # TD
                            for time in range(TD_right_time, min(TD_right_time + TD_right_time, g_number_of_simulation_intervals)):
                                temp_time = min(time, g_number_of_simulation_intervals - 1)
                                sum_of_multipliers = sum_of_multipliers + g_link_list[link].time_dependent_arrival_LR_multiplier_matrix[temp_time]

                            temporary_label_cost = m_label_cost[from_node][t] + travel_cost 
                            # + sum_of_multipliers
                            
                            if (temporary_label_cost < m_label_cost[to_node][new_to_node_arrival_time]):
                                # Update label cost and predecessor
                                m_label_cost[to_node][new_to_node_arrival_time] = temporary_label_cost
                                m_node_predecessor[to_node][new_to_node_arrival_time] = from_node
                                m_time_predecessor[to_node][new_to_node_arrival_time] = t

    return total_cost, m_label_cost, m_node_predecessor, m_time_predecessor

# Backtrace for LR upper bound
def find_ST_path_for_agents(arg, LR_iteration):
    agent_no = arg[0]
    internal_origin_node_seq_no = arg[1]
    departure_time_beginning = arg[2]
    arrival_time_ending = arg[3]
    m_origin_node = internal_origin_node_seq_no
    m_departure_time_beginning = departure_time_beginning 
    m_arrival_time_ending = arrival_time_ending

    reverse_path_node_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    reverse_path_time_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    reverse_path_cost_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS

    path_node_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    path_link_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    path_time_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS
    path_cost_seq = [-1] * MAX_NUMBER_OF_TIME_INTERVALS

    return_value, m_label_cost, m_node_predecessor, m_time_predecessor = \
        optimal_ST_dynamic_programming(arg, LR_iteration)
    total_cost = MAX_LABEL_COST

    # Backtrace in the shortest path tree
    p_agent = g_agent_list[agent_no]
    p_agent.path_link_seq_no_vector_upper_bound.clear()
    p_agent.path_timestamp_vector_upper_bound.clear()
    p_agent.path_node_id_vector_upper_bound.clear()
    p_agent.path_new_link_id_vector_upper_bound.clear()
    p_agent.path_link_TA_vector_upper_bound.clear()
    p_agent.path_link_TD_vector_upper_bound.clear()
    p_agent.path_new_node_id_vector_upper_bound.clear()
    p_agent.path_new_timestamp_vector_upper_bound.clear()

    if(return_value == -1):
        print('agent ' + p_agent.agent_id + ' can not find the ST path')
        return

    destination_node_seq_no = g_internal_node_seq_no_dict[p_agent.d_node_id]
    min_cost_time_index = m_arrival_time_ending

    total_cost = m_label_cost[destination_node_seq_no][min_cost_time_index]

    for t in range(m_arrival_time_ending, m_departure_time_beginning, -1):
        if(m_label_cost[destination_node_seq_no][t] <= total_cost):
            min_cost_time_index = t
            total_cost = m_label_cost[destination_node_seq_no][t]

    # Initialize the reversed array
    node_size = 0
    reverse_path_node_seq[node_size] = destination_node_seq_no
    if (p_agent.agent_type != 'micro agent'):
        reverse_path_time_seq[node_size] = min_cost_time_index
        reverse_path_cost_seq[node_size] = m_label_cost[destination_node_seq_no][min_cost_time_index]
    if (p_agent.agent_type == 'micro agent'):
        reverse_path_time_seq[node_size] = arrival_time_ending
        reverse_path_cost_seq[node_size] = m_label_cost[destination_node_seq_no][arrival_time_ending]

    node_size = node_size + 1

    pred_node = m_node_predecessor[destination_node_seq_no][min_cost_time_index]
    pred_time = m_time_predecessor[destination_node_seq_no][min_cost_time_index]

    while((pred_node != -1) & (node_size < MAX_NUMBER_OF_TIME_INTERVALS)):
        # Scanning backward in the predecessor array
        reverse_path_node_seq[node_size] = pred_node
        reverse_path_time_seq[node_size] = pred_time
        reverse_path_cost_seq[node_size] = m_label_cost[pred_node][pred_time]
        
        node_size = node_size + 1
        
        pred_node_record = pred_node
        pred_time_record = pred_time
        
        pred_node = m_node_predecessor[pred_node_record][pred_time_record]
        pred_time = m_time_predecessor[pred_node_record][pred_time_record]
    
    # Reverse the sequence
    path_node_seq = list(filter(lambda x:x!=-1,reverse_path_node_seq))
    path_node_seq.reverse()
    path_time_seq = list(filter(lambda x:x!=-1,reverse_path_time_seq))
    path_time_seq.reverse()
    path_cost_seq = list(filter(lambda x:x!=-1,reverse_path_cost_seq))
    path_cost_seq.reverse()

    for temp_i in range(0, node_size): # Store node_upper_bound into agent
        p_agent.path_node_id_vector_upper_bound.append(list(g_internal_node_seq_no_dict.keys())[list(g_internal_node_seq_no_dict.values()).index(path_node_seq[temp_i])])
        p_agent.path_timestamp_vector_upper_bound.append(path_time_seq[temp_i])

    for i in range(0, node_size-1):
        for link in g_link_list:
            if ((path_node_seq[i] == link.from_node_seq_no) & (path_node_seq[i+1] == link.to_node_seq_no)):
                 p_agent.path_link_seq_no_vector_upper_bound.append(link.link_seq_no)
                 break
    
    travel_time_return_value = path_time_seq[node_size - 1] - path_time_seq[0] # Running time
    path_number_of_nodes = node_size

    # Distinguish the arrival time and departure time for each node
    temp_node_id = p_agent.path_node_id_vector_upper_bound[0]
    node_arrival_time = p_agent.path_timestamp_vector_upper_bound[0]
    node_departure_time = p_agent.path_timestamp_vector_upper_bound[0]

    for n in range(0, len(p_agent.path_node_id_vector_upper_bound)-1):
        if (node_departure_time == p_agent.path_timestamp_vector_upper_bound[n+1]): # Waitting at here
            node_departure_time = p_agent.path_timestamp_vector_upper_bound[n+1]
        else: #Going to next station
            p_agent.path_new_node_id_vector_upper_bound.append(temp_node_id)
            p_agent.path_new_timestamp_vector_upper_bound.append(node_arrival_time)
            p_agent.path_new_timestamp_vector_upper_bound.append(node_departure_time)

            temp_node_id = p_agent.path_node_id_vector_upper_bound[n+1]
            node_arrival_time = p_agent.path_timestamp_vector_upper_bound[n+1]
            node_departure_time = p_agent.path_timestamp_vector_upper_bound[n+1]
        
    p_agent.path_new_node_id_vector_upper_bound.append(temp_node_id)
    p_agent.path_new_timestamp_vector_upper_bound.append(node_arrival_time)
    p_agent.path_new_timestamp_vector_upper_bound.append(node_departure_time)

    # Get the path (link) vector, TA and TD vector
    temp_number = 1
    for n in range(0, len(p_agent.path_new_node_id_vector_upper_bound)-1):
        p_agent.path_link_TA_vector_upper_bound.append(p_agent.path_new_timestamp_vector_upper_bound[temp_number])
        temp_number = temp_number + 2

    temp_number = 3
    for n in range(1, len(p_agent.path_new_node_id_vector_upper_bound)):
        p_agent.path_link_TD_vector_upper_bound.append(p_agent.path_new_timestamp_vector_upper_bound[temp_number])
        temp_number = temp_number + 2

    temp = len(p_agent.path_new_node_id_vector_upper_bound)
 
    for n in range(0, len(p_agent.path_new_node_id_vector_upper_bound)-1):
        from_node_no = p_agent.path_new_node_id_vector_upper_bound[n]
        to_node_no = p_agent.path_new_node_id_vector_upper_bound[n+1]

        for link in g_link_list:
            if ((from_node_no == link.from_node_id) & (to_node_no == link.to_node_id)):
                temp_link_no = link.link_seq_no
                p_agent.path_new_link_id_vector_upper_bound.append(temp_link_no)

    # How to deal with conflicts? Scanning the shortest path to compute the time dependent link volume.
    for l in range(0, len(p_agent.path_new_link_id_vector_upper_bound)):
        link_seq_no = p_agent.path_new_link_id_vector_upper_bound[l]

        TA_left_time = p_agent.path_link_TA_vector_upper_bound[l]
        TD_right_time = p_agent.path_link_TD_vector_upper_bound[l]
        TA_left_time_headway = 0
        TD_right_time_headway = 0
        next_link_no = 0
        former_link_no = 0

        if ((g_link_list[link_seq_no].link_type != 11) & (g_link_list[link_seq_no].link_type != 12)):
            # "11" means "siding track" type and "12" means "dummy track" type
            # Compute left_time_headway and right_time_headway
            if(l == 0): # First section in this solution
                TA_left_time_headway = g_departure_headway_stop # departure event
                next_link_no = p_agent.path_new_link_id_vector_upper_bound[l+1]
                if (g_link_list[next_link_no].link_type == 11): # next->arrival event
                    # TD_right_time_headway = g_arrival_headway_stop
                    TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                else:
                    TD_right_time_headway = g_arrival_headway_passing

            elif(l == len(p_agent.path_new_link_id_vector_upper_bound) - 1): # Last section
                # the last one is dummuy link, not the destination link
                TD_right_time_headway = g_arrival_headway_stop
                TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                former_link_no = p_agent.path_new_link_id_vector_upper_bound[l-1]
                if(g_link_list[former_link_no].link_type == 11): # former->departure event
                    # TA_left_time_headway = g_departure_headway_stop
                    TA_left_time_headway = g_agent_list[agent_no].headway[former_link_no]
                else: # passing event
                    TA_left_time_headway = g_departure_headway_passing
            else: # intermediate section
                next_link_no = p_agent.path_new_link_id_vector_upper_bound[l+1]
                if(g_link_list[next_link_no].link_type == 11): # next->arrival event
                    # TD_right_time_headway = g_arrival_headway_stop
                    TD_right_time_headway = g_agent_list[agent_no].headway[next_link_no]
                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                # elif (g_link_list[next_link_no].link_type == 13):
                #     TD_right_time_headway = g_agent_list[agent_no].max_link_travel_time[next_link_no] + g_arrival_headway_stop
                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                else: # passing event
                    TD_right_time_headway = g_arrival_headway_passing

                former_link_no = p_agent.path_new_link_id_vector_upper_bound[l-1]
                if(g_link_list[former_link_no].link_type == 11): # former->departure event
                    # TA_left_time_headway = g_departure_headway_stop
                    TA_left_time_headway = g_agent_list[agent_no].headway[former_link_no]
                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                # elif(g_link_list[former_link_no].link_type == 13):
                #     TA_left_time_headway = g_agent_list[agent_no].max_link_travel_time[former_link_no] + g_departure_headway_stop
                # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #                   
                else: # passing event
                    TA_left_time_headway = g_departure_headway_passing

        if (g_link_list[link_seq_no].link_type == 13):
            TA_left_time_headway = g_agent_list[agent_no].max_link_travel_time[link_seq_no] + g_arrival_headway_stop
            TD_right_time_headway = g_agent_list[agent_no].max_link_travel_time[link_seq_no] + g_departure_headway_stop
            
            # Marking same link
            for temp_time in range(max(0, TA_left_time - TA_left_time_headway), min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                time = min(temp_time, g_number_of_simulation_intervals)
                g_link_list[link_seq_no].departure_dependent_travel_time_matrix[time] = \
                    g_link_list[link_seq_no].departure_dependent_travel_time_matrix[time] - 1 # => equal to -1

            for temp_time in range(max(0, TD_right_time - TD_right_time_headway), min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                time = min(temp_time, g_number_of_simulation_intervals)
                g_link_list[link_seq_no].arrival_dependent_travel_time_matrix[time] = \
                    g_link_list[link_seq_no].arrival_dependent_travel_time_matrix[time] - 1 # => equal to -1
        

            # Marking oppo link
            for temp_time in range(max(0, TA_left_time - TA_left_time_headway), min(TD_right_time + g_departure_headway_stop, g_number_of_simulation_intervals)):
                time = min(temp_time, g_number_of_simulation_intervals)
                if (g_link_list[link_seq_no].same_link_id != 1000):
                    same_link_no = g_internal_link_seq_no_dict[g_link_list[link_seq_no].same_link_id]
                    g_link_list[same_link_no].departure_dependent_travel_time_matrix[time] = \
                        g_link_list[same_link_no].departure_dependent_travel_time_matrix[time] - 1

            for temp_time in range(max(0, TA_left_time - g_arrival_headway_stop), min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                time = min(temp_time, g_number_of_simulation_intervals)
                if (g_link_list[link_seq_no].same_link_id != 1000):
                    same_link_no = g_internal_link_seq_no_dict[g_link_list[link_seq_no].same_link_id]
                    g_link_list[same_link_no].arrival_dependent_travel_time_matrix[time] = \
                        g_link_list[same_link_no].arrival_dependent_travel_time_matrix[time] - 1

        if (g_link_list[link_seq_no].link_type != 13):
            # Marking the used arc
            for temp_time in range(TA_left_time, min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                time = min(temp_time, g_number_of_simulation_intervals)
                g_link_list[link_seq_no].departure_dependent_travel_time_matrix[time] = \
                    g_link_list[link_seq_no].departure_dependent_travel_time_matrix[time] - 1 # => equal to -1

                if (g_link_list[link_seq_no].same_link_id != 1000) and (p_agent.agent_type != 'micro agent'):
                    same_link_no = g_internal_link_seq_no_dict[g_link_list[link_seq_no].same_link_id]
                    g_link_list[same_link_no].departure_dependent_travel_time_matrix[time] = \
                        g_link_list[same_link_no].departure_dependent_travel_time_matrix[time] - 1

                if (TA_left_time_headway > 3) and (p_agent.agent_type != 'micro agent'):
                    if(g_link_list[link_seq_no].oppo_link_id != 1000):                
                        oppo_link_no = g_internal_link_seq_no_dict[g_link_list[link_seq_no].oppo_link_id]
                        g_link_list[oppo_link_no].departure_dependent_travel_time_matrix[time] = \
                            g_link_list[oppo_link_no].departure_dependent_travel_time_matrix[time] - 1 # => equal to -1

            for temp_time in range(TD_right_time, min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                time = min(temp_time, g_number_of_simulation_intervals)
                g_link_list[link_seq_no].arrival_dependent_travel_time_matrix[time] = \
                    g_link_list[link_seq_no].arrival_dependent_travel_time_matrix[time] - 1 # => equal to -1

                if (g_link_list[link_seq_no].same_link_id != 1000) and (p_agent.agent_type != 'micro agent'):
                    same_link_no = g_internal_link_seq_no_dict[g_link_list[link_seq_no].same_link_id]
                    g_link_list[same_link_no].arrival_dependent_travel_time_matrix[time] = \
                        g_link_list[same_link_no].arrival_dependent_travel_time_matrix[time] - 1

                if (TD_right_time_headway > 3) and (p_agent.agent_type != 'micro agent'):
                    if(g_link_list[link_seq_no].oppo_link_id != 1000):
                        oppo_link_no = g_internal_link_seq_no_dict[g_link_list[link_seq_no].oppo_link_id]
                        g_link_list[oppo_link_no].arrival_dependent_travel_time_matrix[time] = \
                            g_link_list[oppo_link_no].arrival_dependent_travel_time_matrix[time] - 1 # => equal to -1


    g_agent_list[agent_no] = p_agent
    return total_cost

def g_UpdateResourceUsageStatus():
    # Reset the link count and usage flag
    for link in g_link_list:
        for t in range(0,g_number_of_simulation_intervals):
            link.g_link_time_departure_visit_counts[t] = 0
            link.g_link_time_arrival_visit_counts[t] = 0
            for a in range(0, g_number_of_agents):
                link.g_link_time_departure_train_visit_flag[t][a] = 0
                link.g_link_time_arrival_train_visit_flag[t][a] = 0
    
    # Update the link visit count
    for agent in g_agent_list:
        for l in range(0, len(agent.path_new_link_id_vector)): #for each link in each agent
            TA_left_time = agent.path_link_TA_vector[l]
            TD_right_time = agent.path_link_TA_vector[l]
            TA_left_time_headway = 0
            TD_right_time_headway = 0
            next_link_no = 0
            former_link_no = 0

            link_seq_no = agent.path_new_link_id_vector[l]
            # if (g_link_list[link_seq_no].link_type != 12):
            if ((g_link_list[link_seq_no].link_type != 11) & (g_link_list[link_seq_no].link_type != 12)):
                # "11" means "siding track" type and "12" means "dummy track" type
                # Scanning left_time_headway and right_time_headway
                if(l == 0): # First section
                    TA_left_time_headway = g_departure_headway_stop # departure event
                    next_link_no = agent.path_new_link_id_vector[l+1]
                    if (g_link_list[next_link_no].link_type == 11): # next->arrival event
                        # TD_right_time_headway = g_arrival_headway_stop
                        TD_right_time_headway = g_agent_list[agent.agent_id].headway[next_link_no]
                    else:
                        TD_right_time_headway = g_arrival_headway_passing

                elif(l == len(agent.path_new_link_id_vector) - 1): # Last section
                    TD_right_time_headway = g_arrival_headway_stop
                    former_link_no = agent.path_new_link_id_vector[l-1]
                    if(g_link_list[former_link_no].link_type == 11): # former->departure event
                        # TA_left_time_headway = g_departure_headway_stop
                        TA_left_time_headway = g_agent_list[agent.agent_id].headway[former_link_no]
                    else: # passing event
                        TA_left_time_headway = g_departure_headway_passing
                
                else: # intermediate section
                    next_link_no = agent.path_new_link_id_vector[l+1]
                    if(g_link_list[next_link_no].link_type == 11): # next->arrival event
                        # TD_right_time_headway = g_arrival_headway_stop
                        TD_right_time_headway = g_agent_list[agent.agent_id].headway[next_link_no]
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    # elif (g_link_list[next_link_no].link_type == 13):
                    #     TD_right_time_headway = g_agent_list[agent.agent_id].max_link_travel_time[next_link_no] + g_arrival_headway_stop
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    else: # passing event
                        TD_right_time_headway = g_arrival_headway_passing
                                
                    former_link_no = agent.path_new_link_id_vector[l-1]
                    if(g_link_list[former_link_no].link_type == 11): # former->departure event
                        # TA_left_time_headway = g_departure_headway_stop
                        TA_left_time_headway = g_agent_list[agent.agent_id].headway[former_link_no]
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    # elif (g_link_list[next_link_no].link_type == 13):
                    #     TA_left_time_headway = g_agent_list[agent.agent_id].max_link_travel_time[former_link_no] + g_departure_headway_stop
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    else: # passing event
                        TA_left_time_headway = g_departure_headway_passing
                            
                # Counting
                for t in range(TA_left_time, min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                    if(g_link_list[link_seq_no].g_link_time_departure_train_visit_flag[t][agent.agent_id] == 0):
                        time = min(t, g_number_of_simulation_intervals-1)
                        g_link_list[link_seq_no].g_link_time_departure_visit_counts[time] = \
                            g_link_list[link_seq_no].g_link_time_departure_visit_counts[time] + 1
                        g_link_list[link_seq_no].g_link_time_departure_train_visit_flag[time][agent.agent_id] = 1
                
                for t in range(TD_right_time, min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                    if(g_link_list[link_seq_no].g_link_time_departure_train_visit_flag[t][agent.agent_id] == 0):
                        time = min(t, g_number_of_simulation_intervals-1)
                        g_link_list[link_seq_no].g_link_time_arrival_visit_counts[time] = \
                            g_link_list[link_seq_no].g_link_time_arrival_visit_counts[time] + 1
                        g_link_list[link_seq_no].g_link_time_arrival_train_visit_flag[time][agent.agent_id] = 1
                
def g_DeduceToProblemFeasibleSolution(g_agent_sequence, LR_iteration):
    # Getting the upper bound solution based on agent sequence
    for i in g_agent_sequence:
        agent_no = i
        internal_origin_node_seq_no = g_internal_node_seq_no_dict[g_agent_list[i].o_node_id]
        departure_time_beginning = g_agent_list[i].earliest_departure_time
        arrival_time_ending = g_number_of_intervals_in_master_schedule - 1        
        try:
            if (g_agent_list[i].latest_arrival_time != 0):
                arrival_time_ending = g_agent_list[i].latest_arrival_time
        except:
            temp_value = 0

            
        arg = []
        arg.append(agent_no)
        arg.append(internal_origin_node_seq_no)
        arg.append(departure_time_beginning)
        arg.append(arrival_time_ending)

        find_ST_path_for_agents(arg, LR_iteration)

def g_train_ranking(LR_iteration):
    g_agent_sequence = []
    g_agent_DeviationRatio = []
    for agent in g_agent_list:
        # # # # # #
        free_trip_time = agent.free_flow_travel_time
        actual_trip_time = agent.travel_time_in_dual_solution
        # # # # # #
        g_agent_DeviationRatio.append(free_trip_time) # Choose ranking key as total travel time
    
    g_agent_sequence = sorted(range(len(g_agent_DeviationRatio)), key=lambda k: g_agent_DeviationRatio[k], reverse = True)
    return g_agent_sequence

def g_upper_bound_solution_feasibility_check():
    # Reset the link visit count and usage flag
    for link in g_link_list:
        link.g_link_time_departure_visit_counts_in_upper_bound = [0] * g_number_of_simulation_intervals
        link.g_link_time_arrival_visit_counts_in_upper_bound = [0] * g_number_of_simulation_intervals
        link.g_link_time_departure_train_visit_flag_in_upper_bound.clear()
        link.g_link_time_arrival_train_visit_flag_in_upper_bound.clear()

        for t in range(0, g_number_of_simulation_intervals):
            link.g_link_time_departure_train_visit_flag_in_upper_bound.append([])
            link.g_link_time_arrival_train_visit_flag_in_upper_bound.append([])
            for a in range(0, g_number_of_agents):
                link.g_link_time_departure_train_visit_flag_in_upper_bound[t].append(0)
                link.g_link_time_arrival_train_visit_flag_in_upper_bound[t].append(0)
    
    # Update total link visit count based on each agent
    for p_agent in g_agent_list:
        a = p_agent.agent_id
        if (len(p_agent.path_new_link_id_vector_upper_bound) == 0):
            return 0 
        for l in range(0, len(p_agent.path_new_link_id_vector_upper_bound)): # for each link in this agent
            link_seq_no = p_agent.path_new_link_id_vector_upper_bound[l]

            TA_left_time = p_agent.path_link_TA_vector_upper_bound[l]
            TD_right_time = p_agent.path_link_TD_vector_upper_bound[l]
            TA_left_time_headway = 0
            TD_right_time_headway = 0
            next_link_no = 0
            former_link_no = 0

            # if (g_link_list[link_seq_no].link_type != 12):
            if ((g_link_list[link_seq_no].link_type != 11) & (g_link_list[link_seq_no].link_type != 12)):
                # "11" means "siding track" type and "12" means "dummy track" type
                # Compute left_time_headway and right_time_headway
                if(l == 0): # First section in this solution
                    # TA_left_time_headway = g_departure_headway_stop # departure event
                    TA_left_time_headway = g_agent_list[p_agent.agent_id].headway[former_link_no]
                    next_link_no = p_agent.path_new_link_id_vector_upper_bound[l+1]
                    if (g_link_list[next_link_no].link_type == 11): # next->arrival event
                        # TD_right_time_headway = g_arrival_headway_stop
                        TD_right_time_headway = g_agent_list[p_agent.agent_id].headway[next_link_no]
                    else:
                        TD_right_time_headway = g_arrival_headway_passing

                elif(l == len(p_agent.path_new_link_id_vector_upper_bound) - 1): # Last section
                    # the last one is dummuy link, not the destination link
                    TD_right_time_headway = g_arrival_headway_stop
                    former_link_no = p_agent.path_new_link_id_vector_upper_bound[l-1]
                    if(g_link_list[former_link_no].link_type == 11): # former->departure event
                        # TA_left_time_headway = g_departure_headway_stop
                        TA_left_time_headway = g_agent_list[p_agent.agent_id].headway[former_link_no]
                    else: # passing event
                        TA_left_time_headway = g_departure_headway_passing
                else: # intermediate section
                    next_link_no = p_agent.path_new_link_id_vector_upper_bound[l+1]
                    if(g_link_list[next_link_no].link_type == 11): # next->arrival event
                        # TD_right_time_headway = g_arrival_headway_stop
                        TD_right_time_headway = g_agent_list[p_agent.agent_id].headway[next_link_no]
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    # elif (g_link_list[next_link_no].link_type == 13):
                    #     TD_right_time_headway = g_agent_list[p_agent.agent_id].max_link_travel_time[next_link_no] + g_arrival_headway_stop
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    else: # passing event
                        TD_right_time_headway = g_arrival_headway_passing

                    former_link_no = p_agent.path_new_link_id_vector_upper_bound[l-1]
                    if(g_link_list[former_link_no].link_type == 11): # former->departure event
                        # TA_left_time_headway = g_departure_headway_stop
                        TA_left_time_headway = g_agent_list[p_agent.agent_id].headway[former_link_no]
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    # elif (g_link_list[former_link_no].link_type == 13):
                    #     TA_left_time_headway = g_agent_list[p_agent.agent_id].max_link_travel_time[former_link_no] + g_departure_headway_stop
                    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
                    else: # passing event
                        TA_left_time_headway = g_departure_headway_passing

                for time in range(TA_left_time, min(TA_left_time + TA_left_time_headway, g_number_of_simulation_intervals)):
                    if(g_link_list[link_seq_no].g_link_time_departure_train_visit_flag_in_upper_bound[time][a] == 0):
                        temp_time = min(time, g_number_of_simulation_intervals)
                        g_link_list[link_seq_no].g_link_time_departure_visit_counts_in_upper_bound[temp_time] = \
                            g_link_list[link_seq_no].g_link_time_departure_visit_counts_in_upper_bound[temp_time] + 1
                        
                        g_link_list[link_seq_no].g_link_time_departure_train_visit_flag_in_upper_bound[temp_time][a] = 1

                for time in range(TD_right_time, min(TD_right_time + TD_right_time_headway, g_number_of_simulation_intervals)):
                    if(g_link_list[link_seq_no].g_link_time_arrival_train_visit_flag_in_upper_bound[time][a] == 0):
                        temp_time = min(time, g_number_of_simulation_intervals)
                        g_link_list[link_seq_no].g_link_time_arrival_visit_counts_in_upper_bound[temp_time] = \
                            g_link_list[link_seq_no].g_link_time_arrival_visit_counts_in_upper_bound[temp_time] + 1

                        g_link_list[link_seq_no].g_link_time_arrival_train_visit_flag_in_upper_bound[temp_time][a] = 1          

        # Check capacity
        bFeasibleFlag = 1
        for l in range(0, len(g_link_list)):
            # if(g_link_list[l].direction == 0): # Why?
            # if (g_link_list[link_seq_no].link_type != 12):
            if ((g_link_list[link_seq_no].link_type != 11) & (g_link_list[link_seq_no].link_type != 12)):
                for t in range(0, g_number_of_simulation_intervals):
                    
                    link_visit_counts = g_link_list[l].g_link_time_departure_visit_counts_in_upper_bound[t]
                    if (g_link_list[l].same_link_id != 1000):
                        same_link_no = g_internal_link_seq_no_dict[g_link_list[l].same_link_id]
                        link_visit_counts += g_link_list[same_link_no].g_link_time_departure_visit_counts_in_upper_bound[t]
                        
                    if (link_visit_counts > g_link_list[l].time_depedent_capacity_matrix[t]):
                        bFeasibleFlag = 0
                        break

                    link_visit_counts = g_link_list[l].g_link_time_arrival_visit_counts_in_upper_bound[t]
                    if (g_link_list[l].same_link_id != 1000):
                        same_link_no = g_internal_link_seq_no_dict[g_link_list[l].same_link_id]
                        link_visit_counts += g_link_list[same_link_no].g_link_time_arrival_visit_counts_in_upper_bound[t]

                    if (link_visit_counts > g_link_list[l].time_depedent_capacity_matrix[t]):
                        bFeasibleFlag = 0
                        break
                    
                    if (bFeasibleFlag == 0):
                        break
                if (bFeasibleFlag == 0):
                    break   
        return bFeasibleFlag               
         
def g_Transfer_Macro_to_Micro():
    for workbench_no in range(1, len(g_workbench_list)):
        g_workbench_list[workbench_no].g_agent_list = []

    # Output agent.csv for each station
    for station_no in g_station_list:
        os.chdir(file_chdir + '\\micro_network\\station_w' + str(station_no))
        with open('input_agent.csv', 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow([ \
            'agent_id',
            'from_origin_node_id',
            'to_destination_node_id',
            'earliest_departure_time',
            'latest_arrival_time',
            'frequency',
            'direction',
            'speed_grade',
            'set_of_allowed_links',
            'min_link_travel_time',
            'max_link_travel_time',
            'headway'])

    for agent in g_agent_list:
        for link_no in range(0, len(agent.g_output_link_NO_vector)):
            link = agent.g_output_link_NO_vector[link_no]
            if(g_link_list[link].name != ''):
                if (g_link_list[link].link_type == 11):
                    if ((agent.g_output_link_TD_vector[link_no] - agent.g_output_link_TA_vector[link_no]) != 1): # Stopping
                        station_name = g_link_list[link].name 

                        os.chdir(file_chdir + '\\micro_network\\station_' + str(station_name))
                        with open('input_agent.csv', 'a', newline='') as outfile:
                            writer = csv.writer(outfile)
                            key = (station_name, 'stop', g_link_list[link].from_node_id, g_link_list[link].to_node_id)
                            str_allowed_links = ";".join([str(x) for x in g_agent_map_dict[key].allowed_links_list])     
                            str_min_link_travel_time = ";".join([str(x) for x in g_agent_map_dict[key].min_link_travel_time])
                            str_max_link_travel_time =  ";".join([str(x) for x in g_agent_map_dict[key].max_link_travel_time])                       
                            str_headway = ";".join([str(x) for x in g_agent_map_dict[key].headway])

                            writer.writerow([ \
                                agent.agent_id,
                                g_agent_map_dict[key].micro_from_node_id,
                                g_agent_map_dict[key].micro_to_node_id,
                                agent.g_output_link_TA_vector[link_no] * 60,
                                agent.g_output_link_TD_vector[link_no] * 60,
                                str(1),
                                str(1),
                                str(1),
                                str_allowed_links,
                                str_min_link_travel_time,
                                str_max_link_travel_time,
                                str_headway])
                        
                        import re
                        station_temp = int(re.sub("\D", "", station_name))
                        workbench_index = g_station_list.index(station_temp) + 1
                        micro_agent = Agent()
                        micro_agent.agent_id = agent.agent_id
                        micro_agent.o_node_id = g_agent_map_dict[key].micro_from_node_id
                        micro_agent.d_node_id = g_agent_map_dict[key].micro_to_node_id
                        micro_agent.o_node_seq_no = g_workbench_list[workbench_index].g_internal_node_seq_no_dict[micro_agent.o_node_id]
                        micro_agent.d_node_seq_no = g_workbench_list[workbench_index].g_internal_node_seq_no_dict[micro_agent.d_node_id]

                        micro_agent.earliest_departure_time = agent.g_output_link_TA_vector[link_no] * 60
                        micro_agent.latest_arrival_time = agent.g_output_link_TD_vector[link_no] * 60
                        
                        micro_agent.set_of_allowed_links = list(map(int, g_agent_map_dict[key].allowed_links_list))
                        micro_agent.min_link_travel_time = list(map(int, g_agent_map_dict[key].min_link_travel_time))
                        micro_agent.max_link_travel_time = list(map(int, g_agent_map_dict[key].max_link_travel_time))
                        micro_agent.headway = list(map(int, g_agent_map_dict[key].headway))

                        micro_agent.agent_type = 'micro agent'
                        # stopping_time_interval = micro_agent.latest_arrival_time - micro_agent.earliest_departure_time - 2 * 60
                        # for link_seq_no in micro_agent.set_of_allowed_links:
                        #     if (g_workbench_list[workbench_index].g_link_list[link_seq_no].link_type == 11):
                        #         micro_agent.min_link_travel_time[link_seq_no] = stopping_time_interval - 10
                        #         micro_agent.max_link_travel_time[link_seq_no] = stopping_time_interval + 10

                        g_workbench_list[workbench_index].g_agent_list.append(micro_agent)

                    if ((agent.g_output_link_TD_vector[link_no] - agent.g_output_link_TA_vector[link_no]) == 1): # Passing
                        station_name = g_link_list[link].name 
                        os.chdir(file_chdir + '\\micro_network\\station_' + str(station_name))
                        with open('input_agent.csv', 'a', newline='') as outfile:
                            writer = csv.writer(outfile)
                            key = (station_name, 'over', g_link_list[link].from_node_id, g_link_list[link].to_node_id)
                            str_allowed_links = ";".join([str(x) for x in g_agent_map_dict[key].allowed_links_list])     
                            str_min_link_travel_time = ";".join([str(x) for x in g_agent_map_dict[key].min_link_travel_time])
                            str_max_link_travel_time =  ";".join([str(x) for x in g_agent_map_dict[key].max_link_travel_time])                       
                            str_headway = ";".join([str(x) for x in g_agent_map_dict[key].headway])

                            writer.writerow([ \
                                agent.agent_id,
                                g_agent_map_dict[key].micro_from_node_id,
                                g_agent_map_dict[key].micro_to_node_id,
                                agent.g_output_link_TA_vector[link_no] * 60,
                                agent.g_output_link_TD_vector[link_no] * 60,
                                str(1),
                                str(1),
                                str(1),
                                str_allowed_links,
                                str_min_link_travel_time,
                                str_max_link_travel_time])
                    
                        import re
                        station_temp = int(re.sub("\D", "", station_name))
                        workbench_index = g_station_list.index(station_temp) + 1
                        micro_agent = Agent()
                        micro_agent.agent_id = agent.agent_id
                        micro_agent.o_node_id = g_agent_map_dict[key].micro_from_node_id
                        micro_agent.d_node_id = g_agent_map_dict[key].micro_to_node_id
                        micro_agent.o_node_seq_no = g_workbench_list[workbench_index].g_internal_node_seq_no_dict[micro_agent.o_node_id]
                        micro_agent.d_node_seq_no = g_workbench_list[workbench_index].g_internal_node_seq_no_dict[micro_agent.d_node_id]

                        micro_agent.earliest_departure_time = agent.g_output_link_TA_vector[link_no] * 60
                        micro_agent.latest_arrival_time = agent.g_output_link_TD_vector[link_no] * 60
                        
                        micro_agent.set_of_allowed_links = list(map(int, g_agent_map_dict[key].allowed_links_list))
                        micro_agent.min_link_travel_time = list(map(int, g_agent_map_dict[key].min_link_travel_time))
                        micro_agent.max_link_travel_time = list(map(int, g_agent_map_dict[key].max_link_travel_time))
                        micro_agent.headway =  list(map(int, g_agent_map_dict[key].headway))

                        micro_agent.agent_type = 'micro agent'

                        g_workbench_list[workbench_index].g_agent_list.append(micro_agent)
                
def g_UpdateData_parameter(workbench):
    if (workbench.name == 'macro network'):
        workbench.MAX_NUMBER_OF_TIME_INTERVALS = 150
        workbench.g_number_of_simulation_intervals = workbench.MAX_NUMBER_OF_TIME_INTERVALS 

        workbench.g_initial_LR_multiplier_value = 2
        workbench.g_number_of_LR_iterations = 50    
        workbench.g_minimum_subgradient_step_size = 0.01
        
        # 3min headway 
        workbench.g_departure_headway_stop = 2
        workbench.g_departure_headway_passing = 2
        workbench.g_arrival_headway_stop = 2
        workbench.g_arrival_headway_passing = 2
        workbench.g_departure_headway_stop_DF = 2
        workbench.g_arrival_headway_stop_DF = 2

        # 4min headway 
        # workbench.g_departure_headway_stop = 5
        # workbench.g_departure_headway_passing = 1
        # workbench.g_arrival_headway_stop = 5
        # workbench.g_arrival_headway_passing = 1
        # workbench.g_departure_headway_stop_DF = 1
        # workbench.g_arrival_headway_stop_DF = 1
 
    else:
        workbench.MAX_NUMBER_OF_TIME_INTERVALS = 150 * 60
        workbench.g_number_of_simulation_intervals = workbench.MAX_NUMBER_OF_TIME_INTERVALS 

        workbench.g_initial_LR_multiplier_value = 2
        workbench.g_number_of_LR_iterations = 1
        workbench.g_minimum_subgradient_step_size = 0.001

        workbench.g_departure_headway_stop = 5
        workbench.g_departure_headway_passing = 5
        workbench.g_arrival_headway_stop = 5
        workbench.g_arrival_headway_passing = 5
        workbench.g_departure_headway_stop_DF = 240 + 5
        workbench.g_arrival_headway_stop_DF = 240 + 5
                                       

    global MAX_NUMBER_OF_TIME_INTERVALS
    global g_number_of_simulation_intervals

    global g_initial_LR_multiplier_value
    global g_number_of_LR_iterations
    global g_minimum_subgradient_step_size

    global g_departure_headway_stop
    global g_departure_headway_passing
    global g_arrival_headway_stop
    global g_arrival_headway_passing
    global g_departure_headway_stop_DF
    global g_arrival_headway_stop_DF

    MAX_NUMBER_OF_TIME_INTERVALS = workbench.MAX_NUMBER_OF_TIME_INTERVALS
    g_number_of_simulation_intervals = workbench.g_number_of_simulation_intervals

    g_initial_LR_multiplier_value = workbench.g_initial_LR_multiplier_value
    g_number_of_LR_iterations = workbench.g_number_of_LR_iterations
    g_minimum_subgradient_step_size = workbench.g_minimum_subgradient_step_size

    g_departure_headway_stop = workbench.g_departure_headway_stop
    g_departure_headway_passing = workbench.g_departure_headway_passing
    g_arrival_headway_stop = workbench.g_arrival_headway_stop
    g_arrival_headway_passing = workbench.g_arrival_headway_passing 
    g_departure_headway_stop_DF = workbench.g_departure_headway_stop_DF
    g_arrival_headway_stop_DF = workbench.g_arrival_headway_stop_DF


def g_UpdateData_list(workbench):
    workbench.g_node_list = g_node_list
    workbench.g_link_list = g_link_list
    workbench.g_agent_list = g_agent_list
    workbench.g_number_of_nodes = g_number_of_nodes
    workbench.g_number_of_links = g_number_of_links
    workbench.g_number_of_agents = g_number_of_agents

    workbench.g_upper_bound_list = g_upper_bound_list
    workbench.g_lower_bound_list = g_lower_bound_list

    workbench.g_internal_node_seq_no_dict = g_internal_node_seq_no_dict
    workbench.g_external_node_id_dict = g_external_node_id_dict
    workbench.g_internal_link_seq_no_dict = g_internal_link_seq_no_dict
    workbench.g_external_link_id_dict = g_external_link_id_dict
    workbench.g_link_key_to_seq_no_dict = g_link_key_to_seq_no_dict

    workbench.g_x_axis_unit = g_x_axis_unit
    workbench.g_best_upper_bound = g_best_upper_bound
    workbench.g_best_lower_bound = g_best_lower_bound
    workbench.optimality_gap = optimality_gap
    workbench.freStop_best_feas_Upper_bound_flag = freStop_best_feas_Upper_bound_flag
    workbench.effective_agent = effective_agent

def g_ReadMapData():
    with open('map.csv', 'r') as fp:
        lines = fp.readlines()
        temp = lines[0].strip().split(',')
        station_name = temp.index('station')
        type_name= temp.index('type')
        macro_from_node_id = temp.index('macro_from_node_id')
        macro_to_node_id = temp.index('macro_to_node_id')
        micro_from_node_id = temp.index('micro_from_node_id')
        micro_to_node_id = temp.index('micro_to_node_id')
        set_of_allowed_links = temp.index('set_of_allowed_links')
        min_link_travel_time = temp.index('min_link_travel_time')
        max_link_travel_time = temp.index('max_link_travel_time')
        headway = temp.index('headway')
        for l in lines[1:]:
            l = l.strip().split(',')
            agent = MapAgent()
            agent.station_name = str(l[station_name])
            agent.type_name = str(l[type_name])
            agent.macro_from_node_id = int(l[macro_from_node_id])            
            agent.macro_to_node_id = int(l[macro_to_node_id])
            agent.micro_from_node_id = int(l[micro_from_node_id])
            agent.micro_to_node_id = int(l[micro_to_node_id])
            agent.allowed_links_list = l[set_of_allowed_links].strip().split(';')
            agent.min_link_travel_time = l[min_link_travel_time].strip().split(';')
            agent.max_link_travel_time = l[max_link_travel_time].strip().split(';')
            agent.headway = l[headway].strip().split(';')
            g_agent_map.append(agent)
            key = (agent.station_name, agent.type_name, agent.macro_from_node_id, agent.macro_to_node_id)
            g_agent_map_dict[key] = agent

def g_ReadMacroData():
    print("\n" + "Current Station: Macro Network")
    g_reset_global_parameter()
    workbench = Workbench()
    workbench.name = 'macro network'
    g_UpdateData_parameter(workbench)
    g_ReadInputData()    
    g_UpdateData_list(workbench)
    g_workbench_list.append(workbench)

def g_ReadMicroData():
    file_chdir_temp = os.getcwd()    
    for station in g_station_list:
        # print("\n" + "Current Station: w" + str(station))

        os.chdir(file_chdir_temp + '\\station_w' + str(station))
        g_reset_global_parameter()
        workbench = Workbench()        
        workbench.name = str(station)
        g_UpdateData_parameter(workbench)
        g_ReadInputData()
        g_UpdateData_list(workbench)
        g_workbench_list.append(workbench)

def g_optimization(workbench): 
    g_reset_global_parameter()
    global g_best_lower_bound
    global g_best_upper_bound
    # Parameters
    global MAX_NUMBER_OF_TIME_INTERVALS    
    global g_number_of_simulation_intervals
    global g_initial_LR_multiplier_value    
    global g_number_of_LR_iterations    
    global g_minimum_subgradient_step_size    
    
    MAX_NUMBER_OF_TIME_INTERVALS = workbench.MAX_NUMBER_OF_TIME_INTERVALS
    g_number_of_simulation_intervals = MAX_NUMBER_OF_TIME_INTERVALS 
    g_initial_LR_multiplier_value = workbench.g_initial_LR_multiplier_value
    g_number_of_LR_iterations = workbench.g_number_of_LR_iterations 
    g_minimum_subgradient_step_size = workbench.g_minimum_subgradient_step_size

    # headway requirements
    global g_departure_headway_stop
    global g_departure_headway_passing
    global g_arrival_headway_stop
    global g_arrival_headway_passing
    global g_departure_headway_stop_DF
    global g_arrival_headway_stop_DF

    g_departure_headway_stop = workbench.g_departure_headway_stop
    g_departure_headway_passing = workbench.g_departure_headway_passing
    g_arrival_headway_stop = workbench.g_arrival_headway_stop
    g_arrival_headway_passing = workbench.g_arrival_headway_passing
    g_departure_headway_stop_DF = workbench.g_departure_headway_stop_DF
    g_arrival_headway_stop_DF = workbench.g_arrival_headway_stop_DF

    # print('Step 1: Loading data...')
    global g_node_list
    global g_link_list
    global g_agent_list
    g_node_list = workbench.g_node_list
    g_link_list = workbench.g_link_list
    g_agent_list = workbench.g_agent_list

    global g_number_of_nodes
    global g_number_of_links 
    global g_number_of_agents
    g_number_of_nodes = workbench.g_number_of_nodes
    g_number_of_links = workbench.g_number_of_links
    g_number_of_agents = workbench.g_number_of_agents
    

    global g_upper_bound_list
    global g_lower_bound_list
    g_upper_bound_list = []
    g_lower_bound_list = []


    global g_internal_node_seq_no_dict
    global g_external_node_id_dict
    global g_internal_link_seq_no_dict
    global g_external_link_id_dict
    global g_link_key_to_seq_no_dict

    g_internal_node_seq_no_dict = workbench.g_internal_node_seq_no_dict
    g_external_node_id_dict = workbench.g_external_node_id_dict
    g_internal_link_seq_no_dict = workbench.g_internal_link_seq_no_dict
    g_external_link_id_dict = workbench.g_external_link_id_dict
    g_link_key_to_seq_no_dict = workbench.g_link_key_to_seq_no_dict

    global g_x_axis_unit
    global g_best_upper_bound
    global g_best_lower_bound
    global optimality_gap
    global freStop_best_feas_Upper_bound_flag
    g_x_axis_unit = workbench.g_x_axis_unit
    g_best_upper_bound = 99999
    g_best_lower_bound = -99999
    optimality_gap = None
    freStop_best_feas_Upper_bound_flag = workbench.freStop_best_feas_Upper_bound_flag


    # print('Step 2: Initializing Upperbound Resource Matrix...')
    # initial value of LR multipliers
    for link in g_link_list:
        for t in range(0,g_number_of_simulation_intervals):
            link.time_dependent_departure_LR_multiplier_matrix.append(g_initial_LR_multiplier_value)
            link.time_dependent_arrival_LR_multiplier_matrix.append(g_initial_LR_multiplier_value)
    
    # Reset the link(i, j, t) visit counts and usage flag
    for link in g_link_list:
        for t in range(0,g_number_of_simulation_intervals):
            link.g_link_time_departure_visit_counts.append(0)
            link.g_link_time_arrival_visit_counts.append(0)
            flag = np.zeros(len(g_agent_list))
            flag_temp = flag.tolist()
            link.g_link_time_departure_train_visit_flag.append(flag_temp)
            link.g_link_time_arrival_train_visit_flag.append(flag_temp)


    # print('Step 3: Starting Lagrangain Relaxation Optimization...')
    f = open("train_scheduling.txt", "w")
    # Looping for each Lagrangain Relaxation iteration
    LR_iteration = 0
    global is_upperbound_feasible
    is_upperbound_feasible = 0
    while (LR_iteration <= g_number_of_LR_iterations) or (is_upperbound_feasible == 0):
    # for LR_iteration in range(0,g_number_of_LR_iterations):
        f.write("\n")
        if (LR_iteration == g_number_of_LR_iterations):
            print("Iteration No: " + str(LR_iteration + 1))
        g_stepSize = 1 / (LR_iteration + 1)

        # Keeping the min step_size
        if(g_stepSize < g_minimum_subgradient_step_size):
            g_stepSize = g_minimum_subgradient_step_size
        
        for link in g_link_list:
            # if (link.direction == 0):
            for t in range(0,g_number_of_simulation_intervals):
                
                # Departure
                link_visit_counts = link.g_link_time_departure_visit_counts[t]
                if(link.same_link_id != 1000):
                    same_link_no = g_internal_link_seq_no_dict[link.same_link_id]
                    link_visit_counts = link_visit_counts + g_link_list[same_link_no].g_link_time_departure_visit_counts[t]
                    link.time_dependent_departure_LR_multiplier_matrix[t] = \
                        max(0, link.time_dependent_departure_LR_multiplier_matrix[t] + \
                            g_stepSize * (link_visit_counts - link.capacity))
                    g_link_list[same_link_no].time_dependent_departure_LR_multiplier_matrix[t] = \
                        link.time_dependent_departure_LR_multiplier_matrix[t]
                if(link.same_link_id == 1000):
                    link.time_dependent_departure_LR_multiplier_matrix[t] = \
                        max(0, link.time_dependent_departure_LR_multiplier_matrix[t] + \
                            g_stepSize * (link_visit_counts - link.capacity))

                # Arrival
                link_visit_counts = link.g_link_time_arrival_visit_counts[t]
                if(link.same_link_id != 1000):
                    same_link_no = g_internal_link_seq_no_dict[link.same_link_id]
                    link_visit_counts = link_visit_counts + g_link_list[same_link_no].g_link_time_arrival_visit_counts[t]
                    link.time_dependent_arrival_LR_multiplier_matrix[t] = \
                        max(0, link.time_dependent_arrival_LR_multiplier_matrix[t] + \
                            g_stepSize * (link_visit_counts - link.capacity))
                    g_link_list[same_link_no].time_dependent_arrival_LR_multiplier_matrix[t] = \
                        link.time_dependent_arrival_LR_multiplier_matrix[t]
                if(link.same_link_id == 1000):
                    link.time_dependent_arrival_LR_multiplier_matrix[t] = \
                        max(0, link.time_dependent_arrival_LR_multiplier_matrix[t] + \
                            g_stepSize * (link_visit_counts - link.capacity))
                            

        # LR lower bound
        LR_global_lower_bound = 0
        total_price = 0
        global g_number_of_intervals_in_master_schedule
        g_number_of_intervals_in_master_schedule = MAX_NUMBER_OF_TIME_INTERVALS 

        g_agent_list_clear()
        g_link_list_clear()

        # Setting the line and station capacity
        for link in g_link_list:
            for t in range(0, g_number_of_simulation_intervals):
                if (link.link_type == 11 or link.link_type == 12):
                    link.time_depedent_capacity_matrix.append(4) # capacity = 4 in side track
                else:
                    link.time_depedent_capacity_matrix.append(1)

        if (workbench.name == 'macro network'):
            for agent in g_agent_list:
                agent_no = agent.agent_id
                internal_origin_node_seq_no = g_internal_node_seq_no_dict[agent.o_node_id]
                departure_time_beginning = agent.earliest_departure_time
                arrival_time_ending = g_number_of_intervals_in_master_schedule - 1
                
                arg = []
                arg.append(agent_no)
                arg.append(internal_origin_node_seq_no)
                arg.append(departure_time_beginning)
                arg.append(arrival_time_ending)

                trip_price = find_ST_path_for_agents_LR(arg, LR_iteration)
                total_price = total_price + trip_price

            # LR upper bound
            total_resource_price = 0
            for link in g_link_list:
                for t in range(0,g_number_of_simulation_intervals):
                    total_resource_price = total_resource_price + link.time_dependent_arrival_LR_multiplier_matrix[t]
                    total_resource_price = total_resource_price + link.time_dependent_departure_LR_multiplier_matrix[t]

            LR_global_lower_bound = total_price - total_resource_price
            g_best_lower_bound = max(g_best_lower_bound, LR_global_lower_bound)
            if (workbench.name == 'macro network'):
                if (LR_iteration == g_number_of_LR_iterations):
                    print("Lower bound: " + str(g_best_lower_bound * 60))
            if (workbench.name != 'macro network'):
                print("Lower bound: " + str(g_best_lower_bound))
            g_lower_bound_list.append(g_best_lower_bound)

            # Change into feasible solution
            g_UpdateResourceUsageStatus()

        g_agent_sequence = []
        g_agent_sequence = g_train_ranking(LR_iteration)

        if (workbench.name != 'macro network'):
            # g_agent_sequence = workbench.g_agent_sequence
            g_agent_sequence = [0,2,3,4,5,1]
        # if (workbench.name == 'macro network'):
        #     workbench.g_agent_sequence = g_agent_sequence

        g_DeduceToProblemFeasibleSolution(g_agent_sequence, LR_iteration)
        
        is_upperbound_feasible = 1

        is_upperbound_feasible = g_upper_bound_solution_feasibility_check()

        # Check the dummy link feasible
        global effective_agent
        effective_agent = [1] * len(g_agent_list)
        dummy_link_no = 0
        dummy_link_id = 0
        for agent in g_agent_list:
            if (len(agent.path_new_link_id_vector_upper_bound) == 0):
                print('Not path found in agent ' + str(agent.agent_id))
                effective_agent[agent.agent_id] = 0
                is_upperbound_feasible = 0
            dummy_link_id = agent.set_of_allowed_links[-1] # the last one is dummy link
            dummy_link_no = g_internal_link_seq_no_dict[dummy_link_id]
            if dummy_link_no in agent.path_new_link_id_vector_upper_bound:
                effective_agent[agent.agent_id] = 0
                is_upperbound_feasible = 0
                    # break
        

        # Compute the upper bound 
        total_travel_time = 0
        if(is_upperbound_feasible == 1):
            for agent in g_agent_list:
                n = len(agent.path_new_link_id_vector_upper_bound)
                actual_departure_time = agent.path_link_TA_vector_upper_bound[0]
                actual_arrival_time = agent.path_link_TD_vector_upper_bound[n-1]
                total_travel_time += (actual_arrival_time - actual_departure_time)
            g_best_upper_bound = min(g_best_upper_bound, total_travel_time)
            if (workbench.name == 'macro network') and (LR_iteration == g_number_of_LR_iterations):
                print("Upper bound: " + str(g_best_upper_bound * 60))
                print("Gap = " + str((g_best_upper_bound - g_best_lower_bound) * 100 / g_best_upper_bound) + "%")
            if (workbench.name != 'macro network'):
                print("Upper bound: " + str(g_best_upper_bound))    
            g_upper_bound_list.append(g_best_upper_bound)
        else:
            if (workbench.name == 'macro network'):
                try:
                    g_upper_bound_list.append(g_upper_bound_list[-1])
                except:
                    g_upper_bound_list.append(999)
            else: # For micro raulway network
                for agent in g_agent_list:
                    n = len(agent.path_new_link_id_vector_upper_bound)
                    if (n != 0):
                        actual_departure_time = agent.path_link_TA_vector_upper_bound[0]
                        actual_arrival_time = agent.path_link_TD_vector_upper_bound[n-1]
                        total_travel_time += (actual_arrival_time - actual_departure_time)
                g_best_upper_bound = min(g_best_upper_bound, total_travel_time)
                g_upper_bound_list.append(g_best_upper_bound)
                print("Upper bound: " + str(g_best_upper_bound)) 



        if(is_upperbound_feasible == 1):
            if (LR_iteration == g_number_of_LR_iterations):
                print("Upper bound feasible!")
            str_feasible = 'Feasible!'
        if(is_upperbound_feasible == 0):
            print("Upper bound infeasible!")
            str_feasible = 'Infeasible!'


        # Only get the feaible path
        f.write("iteration_no: " + str(LR_iteration) + "\n")
        # if(total_travel_time <= g_best_upper_bound):
        for agent in g_agent_list:
            f.write("agent: " + str(agent.agent_id) + "\n")
            agent.g_output_link_NO_vector = agent.path_new_link_id_vector_upper_bound
            agent.g_output_link_TD_vector = agent.path_link_TD_vector_upper_bound
            agent.g_output_link_TA_vector = agent.path_link_TA_vector_upper_bound
            for link in range(0, len(agent.g_output_link_NO_vector)):
                f.write("link_no: " + str(agent.g_output_link_NO_vector[link]) + \
                    "  TA: " + str(agent.g_output_link_TA_vector[link]) + \
                    "  TD: " + str(agent.g_output_link_TD_vector[link]) + "\n")
       
        LR_iteration = LR_iteration + 1
        # For macro railway network, we must go on the iteration until a feasible solution is founded!
        # While for micro railway network, we do not need it!
        if (LR_iteration == g_number_of_LR_iterations) and (workbench.name != 'macro network'):
            break

    
    plt.clf()
    plt.title(str_feasible + '\n' + \
        'Current Interation: ' + str(iter) + '\n' + \
        'Current Station: ' + str(workbench.name))
    colors = []
    # Drawing Timetable
    for i in range(72):
        colors.append('#%06X' % randint(0, 0xFFFFFF))
    for agent in g_agent_list:  
        if (effective_agent[agent.agent_id] == 1):     
            t = []
            x = []
            for j in range(0, len(agent.g_output_link_NO_vector)):
                t.append(agent.g_output_link_TA_vector[j])
                t.append(agent.g_output_link_TD_vector[j])
                x_temp_from = g_link_list[agent.g_output_link_NO_vector[j]].from_node_seq_no
                x_temp_to = g_link_list[agent.g_output_link_NO_vector[j]].to_node_seq_no
                x.append(g_node_list[x_temp_from].x_corrd)
                x.append(g_node_list[x_temp_to].x_corrd)
            plt.plot(t, x, color=colors[agent.agent_id])
    # plt.show()

    plt.clf()
    # Drawing Gap Curve
    g_upper_bound_list_second = [i * 60 for i in g_upper_bound_list]
    g_lower_bound_list_second = [i * 60 for i in g_lower_bound_list]
    iteration_list = [i for i in range(0, LR_iteration)]
    plt.rcParams['font.sans-serif']=['SimHei']
    plt.plot(iteration_list, g_upper_bound_list_second, label = 'Upper bound')
    if (workbench.name == 'macro network'):
        plt.plot(iteration_list, g_lower_bound_list_second, label='Lower bound')
    plt.rcParams['axes.unicode_minus'] = False 
    plt.xlabel('迭代次数', fontsize= 20)
    plt.ylabel('目标函数(秒)', fontsize= 20)
    plt.xticks(family = 'Times new roman', fontsize= 20)
    plt.yticks(family = 'Times new roman', fontsize= 20)
    # if (iter == g_outer_iteration) and (workbench.name == 'macro network'):
    # plt.show()


def AdjustFeedback(station_no, macro_workbench, micro_workbench):
    conflict_truple = None
    for link in macro_workbench.g_link_list:
        if (link.name == 'w' + str(station_no)):
            if(link.lanes == 4): # Arrival and departure line instead of passing line
                link_id = link.link_id

                for agent in macro_workbench.g_agent_list:
                    if (link_id in agent.g_output_link_NO_vector):
                        agent_link_index = agent.g_output_link_NO_vector.index(link_id)
                        if(effective_agent[agent.agent_id] == 0) and (agent.direction == macro_workbench.g_link_list[link_id].direction):

                            # Update headway for the train before this train and after this train(for oppsite only)
                            if(agent.direction == 0):
                                for agent_temp in macro_workbench.g_agent_list:
                                    if (agent_temp.direction == 0):
                                        same_former_agent = agent_temp
                                        break
                                for agent_temp in macro_workbench.g_agent_list:
                                    if (agent_temp.direction == 1):
                                        oppo_former_agent = agent_temp
                                        break
                            if(agent.direction == 1):
                                for agent_temp in macro_workbench.g_agent_list:
                                    if (agent_temp.direction == 1):
                                        same_former_agent = agent_temp
                                        break
                                for agent_temp in macro_workbench.g_agent_list:
                                    if (agent_temp.direction == 0):
                                        oppo_former_agent = agent_temp
                                        break
                            same_flag = 0
                            oppo_flag = 0
                            same_former_agent_TA = 0
                            oppo_former_agent_TA = 0

                            for temp_agent in macro_workbench.g_agent_list:
                                # Find the same direction train
                                if (agent.direction == temp_agent.direction):
                                    same_former_agent_link_index = \
                                        same_former_agent.g_output_link_NO_vector.index(link_id)
                                    temp_agent_link_index = temp_agent.g_output_link_NO_vector.index(link_id) 
                                    # Find the latest former agent
                                    if(temp_agent.g_output_link_TA_vector[temp_agent_link_index] < \
                                        agent.g_output_link_TA_vector[agent_link_index]) and \
                                    (temp_agent.g_output_link_TA_vector[temp_agent_link_index] >= \
                                        same_former_agent.g_output_link_TA_vector[same_former_agent_link_index]):
                                        same_former_agent = temp_agent
                                        same_former_agent_TA = temp_agent.g_output_link_TA_vector[temp_agent_link_index]
                                        same_flag = 1

                                # Find the oppo direction train
                                if (link.name == 'w7'): # Only for bi-direction station
                                    if (agent.direction != temp_agent.direction):                                                       
                                        oppo_link_id = macro_workbench.g_link_list[link_id].oppo_link_id                                
                                        oppo_former_agent_link_index = \
                                            oppo_former_agent.g_output_link_NO_vector.index(oppo_link_id)
                                        temp_agent_link_index = temp_agent.g_output_link_NO_vector.index(oppo_link_id)
                                        # Find the latest former agent
                                        if(temp_agent.g_output_link_TA_vector[temp_agent_link_index] < \
                                            agent.g_output_link_TA_vector[agent_link_index]) and \
                                        (temp_agent.g_output_link_TA_vector[temp_agent_link_index] >= \
                                            oppo_former_agent.g_output_link_TA_vector[oppo_former_agent_link_index]):
                                            oppo_former_agent = temp_agent
                                            oppo_former_agent_TA = temp_agent.g_output_link_TA_vector[temp_agent_link_index]
                                            oppo_flag = 1
                                
                        
                            if (same_flag == 1) and (same_former_agent_TA >= oppo_former_agent_TA): 
                                # same_link_id = macro_workbench.g_link_list[link_id].same_link_id
                                same_former_agent.headway[link_id] = same_former_agent.headway[link_id] + 1
                                print('Updated same agent ' + str(same_former_agent.agent_id) + ' headway time: ' + \
                                    str(same_former_agent.headway[link_id]) + ' for link ' + str(link_id))

                            if (oppo_flag == 1) and (same_former_agent_TA < oppo_former_agent_TA): 
                                oppo_link_id = macro_workbench.g_link_list[link_id].oppo_link_id
                                oppo_former_agent.headway[oppo_link_id] = oppo_former_agent.headway[oppo_link_id] + 1
                                print('Updated oppo agent ' + str(oppo_former_agent.agent_id) + ' headway time: ' + \
                                    str(oppo_former_agent.headway[oppo_link_id]) + ' for link ' + str(oppo_link_id))
                                if (micro_workbench.g_agent_sequence.index(oppo_former_agent.agent_id) < \
                                    micro_workbench.g_agent_sequence.index(agent.agent_id)):
                                    conflict_truple = (oppo_former_agent.agent_id, agent.agent_id)
                                else:
                                    conflict_truple = (agent.agent_id, oppo_former_agent.agent_id)
    return conflict_truple

def g_output_drawing_agent_file(feasible_flag, branching_node_no, micro_sequence, conflict_num):
    if (feasible_flag == 1):
        str_file = 'feasible_tree_node_' + str(branching_node_no) + '_seq_' + str(micro_sequence) + '_conflict_' + str(conflict_num) + '.csv'
    else:
        str_file = 'tree_node_' + str(branching_node_no) + '_seq_' + str(micro_sequence) + '_conflict_' + str(conflict_num) + '.csv'
    with open(str_file, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow([ \
            'agent_no',
            'node_name',
            'node_time'])
        for workbench in g_workbench_list[1:]:
            for agent in workbench.g_agent_list:
                for link_no in range(0, len(agent.g_output_link_NO_vector)):
                    from_node_seq_no = workbench.g_link_list[agent.g_output_link_NO_vector[link_no]].from_node_seq_no
                    to_node_seq_no = workbench.g_link_list[agent.g_output_link_NO_vector[link_no]].to_node_seq_no
                    if (workbench.g_node_list[from_node_seq_no].name != ''):
                        line_1 = [ \
                        agent.agent_id,
                        workbench.g_node_list[from_node_seq_no].name,
                        agent.g_output_link_TA_vector[link_no]
                        ]
                        writer.writerow(line_1)
                    
                    if (workbench.g_node_list[to_node_seq_no].name != ''):
                        line_2 = [ \
                        agent.agent_id,
                        workbench.g_node_list[to_node_seq_no].name,
                        agent.g_output_link_TD_vector[link_no]
                        ]
                        writer.writerow(line_2)

def g_init_tree_node():
    treenode = TreeNode()
    train_permutation_list = list(itertools.permutations([i for i in range(g_number_of_agents)]))
    treenode.BB_matrix = [[-1 for i in range(g_number_of_agents)] for i in range(g_number_of_agents)]
    for seq in train_permutation_list:
        flag = 0
        for i, j in zip(seq, seq):
            if (treenode.BB_matrix[i][j] == 1):
                flag = 1
                break
        if (flag == 1):
            continue
        treenode.micro_sequence = seq
        break
    for agent in g_agent_list:
        treenode.agent_headway_list.append(agent.headway)
    BB_tree_queue.put(treenode)


if __name__ == '__main__':
    time_begin = time.time()
    os.chdir("./data_set")

    # Step 1：Read Input Data
    g_ReadMapData()

    file_chdir = os.getcwd()
    os.chdir(file_chdir + '\\macro_network')
    g_ReadMacroData()

    # Initial the root node
    g_init_tree_node()

    os.chdir(file_chdir + '\\micro_network')
    g_ReadMicroData()

    branching_node_no = 0
    while not BB_tree_queue.empty():
        print(BB_tree_queue.qsize())
        current_treenode = BB_tree_queue.get()
        print('Current micro branching node: ' + str(list(current_treenode.micro_sequence)))
        # Update headway and branching sequence
        for agent in g_workbench_list[0].g_agent_list:
            agent.headway = copy.deepcopy(current_treenode.agent_headway_list[agent.agent_id])
        g_workbench_list[0].g_agent_sequence = current_treenode.micro_sequence

        # Step 2: Upper level Optimization         
        g_UpdateData_parameter(g_workbench_list[0])

        os.chdir(file_chdir + '\\macro_network')
        g_optimization(g_workbench_list[0])
        g_UpdateData_list(g_workbench_list[0])
        
        g_Transfer_Macro_to_Micro()

        # Step 3: Lower Level Optimization 
        for station in g_station_list:
            # station = 7
            is_upperbound_feasible = 1
            g_workbench_list[g_station_list.index(station) + 1].g_agent_sequence = current_treenode.micro_sequence
            print("\n" + "Current Station:" + str(station)) 
            g_reset_global_parameter()
            g_UpdateData_parameter(g_workbench_list[g_station_list.index(station) + 1])
            os.chdir(file_chdir + '\\micro_network\\station_w' + str(station))
            g_optimization(g_workbench_list[g_station_list.index(station) + 1])
            g_UpdateData_list(g_workbench_list[g_station_list.index(station) + 1])
            print('conflicts: ' + str(g_workbench_list[g_station_list.index(station) + 1].effective_agent.count(0)))
            if (g_workbench_list[g_station_list.index(station) + 1].effective_agent.count(0) != 0):
                is_upperbound_feasible = 0

            # If not feasible
            conflict_truple = None
            if (is_upperbound_feasible == 0):
                conflict_truple = AdjustFeedback(station, g_workbench_list[0], g_workbench_list[g_station_list.index(station) + 1])

            branching_flag = 0
            if (conflict_truple != None) and (station == 7):
                # Avioid repeat branching                
                if ((current_treenode.BB_matrix[conflict_truple[0]][conflict_truple[1]] == -1) and \
                    (current_treenode.BB_matrix[conflict_truple[1]][conflict_truple[0]] == -1)): 
                    branching_flag = 1
                    print('conflict_truple: ' + str(conflict_truple))
                    left_treenode = TreeNode()
                    left_treenode = copy.deepcopy(current_treenode)
                    left_treenode.BB_matrix[conflict_truple[0]][conflict_truple[1]] = 1
                    train_permutation_list = list(itertools.permutations([i for i in range(g_number_of_agents)]))
                    for seq in train_permutation_list:
                        flag = 0
                        for i in seq:
                            for j in seq:
                                if (left_treenode.BB_matrix[i][j] == 1) and (seq.index(i) > seq.index(j)):
                                    flag = 1
                                    break
                            if (flag == 1):
                                break
                        if (flag == 1):
                            continue
                        left_treenode.micro_sequence = seq
                        break
                    left_treenode.agent_headway_list = []
                    for agent in g_workbench_list[0].g_agent_list:
                        left_treenode.agent_headway_list.append(agent.headway)
                    print('Left node: ' + str(list(left_treenode.micro_sequence)))
                    branching_node_no = branching_node_no + 1
                    left_treenode.tree_node_no = branching_node_no
                    
                    # Bounding
                    bound_flag = 1
                    ending_time = 0
                    for agent in g_workbench_list[0].g_agent_list:
                        ending_time = max(agent.g_output_link_TA_vector)
                    if (ending_time <= g_workbench_list[0].g_number_of_simulation_intervals):
                        BB_tree_queue.put(left_treenode)

                    # Output left branching
                    feasible_flag = 0
                    os.chdir(file_chdir + '\\output_2')
                    g_output_drawing_agent_file(feasible_flag, left_treenode.tree_node_no, left_treenode.micro_sequence, 999)

                    # Right Treenode
                    right_treenode = TreeNode()
                    right_treenode = copy.deepcopy(current_treenode)
                    right_treenode.BB_matrix[conflict_truple[1]][conflict_truple[0]] = 1
                    train_permutation_list = list(itertools.permutations([i for i in range(g_number_of_agents)]))
                    for seq in train_permutation_list:
                        flag = 0
                        for i in seq:
                            for j in seq:
                                if (right_treenode.BB_matrix[i][j] == 1) and (seq.index(i) > seq.index(j)):
                                    flag = 1
                                    break
                            if (flag == 1):
                                break
                        if (flag == 1):
                            continue
                        right_treenode.micro_sequence = seq
                        break

                    right_treenode.agent_headway_list = []
                    for agent in g_workbench_list[0].g_agent_list:
                        headway_temp = []
                        for link in agent.headway:
                            headway_temp.append(3) # Reset the right node
                        right_treenode.agent_headway_list.append(headway_temp)
                    print('Right node: ' + str(list(right_treenode.micro_sequence)))
                    branching_node_no = branching_node_no + 1
                    right_treenode.tree_node_no = branching_node_no
                    BB_tree_queue.put(right_treenode)

                    '''
                    g_workbench_list[g_station_list.index(station) + 1].g_agent_sequence = right_treenode.micro_sequence
                    g_reset_global_parameter()
                    g_UpdateData_parameter(g_workbench_list[g_station_list.index(station) + 1])
                    os.chdir(file_chdir + '\\micro_network\\station_w' + str(station))
                    g_optimization(g_workbench_list[g_station_list.index(station) + 1])
                    g_UpdateData_list(g_workbench_list[g_station_list.index(station) + 1])
                    print('conflicts: ' + str(g_workbench_list[g_station_list.index(station) + 1].effective_agent.count(0)))

                    # If not feasible
                    if (is_upperbound_feasible == 0):
                        # Update headway
                        for agent in g_workbench_list[0].g_agent_list:
                            agent.headway = current_treenode.agent_headway_list[agent.agent_id]
                        conflict_truple_right = AdjustFeedback(station, g_workbench_list[0], g_workbench_list[g_station_list.index(station) + 1])
                    right_treenode.agent_headway_list = []
                    for agent in g_workbench_list[0].g_agent_list:
                        right_treenode.agent_headway_list.append(agent.headway)
                    print('Right node: ' + str(list(right_treenode.micro_sequence)))
                    BB_tree_queue.put(right_treenode)
                    branching_node_no = branching_node_no + 1
                    os.chdir(file_chdir + '\\output')
                    feasible_flag = 0
                    g_output_drawing_agent_file(feasible_flag, branching_node_no, right_treenode.micro_sequence, conflict_truple_right)
                    '''


        conflict_flag = 0
        conlict_num = 0
        for workbench in g_workbench_list[1:]:
            if ((branching_flag == 0) and (workbench.effective_agent.count(0) != 0)):   
                conflict_flag = 1 # No branching but infeasible
                conlict_num = conlict_num + workbench.effective_agent.count(0)
                # break

        if (conflict_flag == 1): # Still exist conflicts
            treenode = TreeNode()
            treenode = copy.deepcopy(current_treenode)
            treenode.agent_headway_list = []
            for agent in g_workbench_list[0].g_agent_list:
                treenode.agent_headway_list.append(agent.headway)

            # Bounding
            bound_flag = 1
            ending_time = 0
            for agent in g_workbench_list[0].g_agent_list:
                ending_time = max(agent.g_output_link_TA_vector)
            if (ending_time <= g_workbench_list[0].g_number_of_simulation_intervals):
                BB_tree_queue.put(treenode)

            # branching_node_no = branching_node_no + 1
            os.chdir(file_chdir + '\\output_2')
            feasible_flag = 0
            g_output_drawing_agent_file(feasible_flag, current_treenode.tree_node_no, treenode.micro_sequence, conlict_num)
        
        
        # Output feasible solution node: Current Node        
        conflict_flag = 0
        for workbench in g_workbench_list[1:]:
            if (workbench.effective_agent.count(0) != 0):   
                conflict_flag = 1
                conlict_num = conlict_num + workbench.effective_agent.count(0)
                # break
        if (conflict_flag == 0):
            os.chdir(file_chdir + '\\output_2')
            # branching_node_no = branching_node_no + 1
            feasible_flag = 1
            g_output_drawing_agent_file(feasible_flag, current_treenode.tree_node_no, current_treenode.micro_sequence, conlict_num)
    
    
    time_end = time.time()
    print("CPU Time: " + str(time_end - time_begin))