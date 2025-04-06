import math
import matplotlib.pyplot as plt
import numpy as np


# SIMULATION PARAMETERS

big_M = 65 # lastest time for last drone to return to depot

scenario = 'PD-SL' # directPD or PD-SL
obj = 1 # 1: min total flight distance, 2: min makespan, 3: min 1+2

cost_drone_per_km = 0.002 # CHF/km
cost_drone_per_minute = 1 # 2 employees, each paid CHF30/hour = CHF0.5/minute
# cost_per_drone_stop = # just need to sum the number of active drone arcs

v_bus = 0.5 # km/minute
t_bus_dwell = 0 # minutes

v_drone = 1 # 57,6 km/h = approx. 1 km/minute, assume faster than bus

# FUNCTIONS
def calc_bus_travel_times(num_stops, stop_df, v_bus):
    bus_travel_times = {}
    for stop_i in range(num_stops - 1):
        stop_j = stop_i + 1
        
        stop_i_x, stop_i_y = stop_df[['stop_x', 'stop_y']].iloc[stop_i].values
        stop_j_x, stop_j_y = stop_df[['stop_x', 'stop_y']].iloc[stop_j].values

        bus_travel_times[(stop_i, stop_j)] = math.sqrt((stop_i_x - stop_j_x)**2 + (stop_i_y - stop_j_y)**2) / v_bus

    for stop_j in range(num_stops - 1):
        stop_i = stop_j + 1
        
        stop_i_x, stop_i_y = stop_df[['stop_x', 'stop_y']].iloc[stop_i].values
        stop_j_x, stop_j_y = stop_df[['stop_x', 'stop_y']].iloc[stop_j].values

        bus_travel_times[(stop_i, stop_j)] = math.sqrt((stop_i_x - stop_j_x)**2 + (stop_i_y - stop_j_y)**2) / v_bus

    return bus_travel_times

def calc_time_bus_arcs(num_nodes, big_M, bus_travel_times, num_tasks, bus_stop_list):

    # Fill out travel time between feasible bus nodes
    time_bus_arcs = np.zeros((num_nodes, num_nodes)) + big_M # to avoid choosing infeasible arcs
    for (s1, s2) in bus_travel_times:
        for task in range(num_tasks):
            time_bus_arcs[(bus_stop_list[s1][task], bus_stop_list[s2][task])] = bus_travel_times[(s1, s2)]

    return time_bus_arcs


def get_node_coords(depot_df, task_df, stop_df, num_drones, bus_stop_list):
    '''
    Returns: 
    dictionary with node ID as key, and x, y coordinates as values.
    node ID is created according to the order in which the nodes are added to the model.
    '''
    nodes_coords_dict = {}
    dict_i = 0

    num_tasks = len(task_df)
    # Add origin depot coordinates
    for _, row in depot_df.iterrows():
        for i in range(num_drones):
            nodes_coords_dict[dict_i] = (row['depot_x'], row['depot_y'])
            dict_i += 1

    # Add task coordinates
    for _, row in task_df.iterrows():
        nodes_coords_dict[dict_i] = (row['pickup_x'], row['pickup_y'])
        nodes_coords_dict[dict_i + num_tasks] = (row['delivery_x'], row['delivery_y'])
        dict_i += 1

    # Move to bus stop node. Above was indexed by pickup IDs
    dict_i = bus_stop_list[0][0]

    # Add bus stop coordinates, replicated for each task
    for _, row in stop_df.iterrows():
        for i in range(num_tasks):
            nodes_coords_dict[dict_i] = (row['stop_x'], row['stop_y'])
            dict_i += 1
    
    # Add destination depot coordinates
    for _, row in depot_df.iterrows():
        for i in range(num_drones):
            nodes_coords_dict[dict_i] = (row['depot_x'], row['depot_y'])
            dict_i += 1    

    return nodes_coords_dict


def calc_drone_arc_dist(num_nodes, big_M, nodes_coords_dict):
    # euclidean distance between all node pairs in nodes_coords_dict
    dist_drone_arcs = np.zeros((num_nodes, num_nodes)) + big_M # to avoid choosing infeasible arcs
    for i in range(num_nodes):
        for j in range(num_nodes):
            dist_drone_arcs[i, j] = math.sqrt((nodes_coords_dict[i][0] - nodes_coords_dict[j][0])**2 + (nodes_coords_dict[i][1] - nodes_coords_dict[j][1])**2)

    # convert dist_drone_arcs to dictionary
    dist_drone_dict = {(i, j): dist_drone_arcs[i, j] for i in range(num_nodes) for j in range(num_nodes)}

    return dist_drone_arcs, dist_drone_dict 


def create_all_nodes(num_depots, num_drones, num_tasks, num_stops): 
    # Create list of all nodes
    num_depots_repl = num_depots * num_drones
    num_nodes = 2 * num_depots_repl + 2 * num_tasks + num_stops * num_tasks
    nodes_list = [*range(num_nodes)]

    # origin and destination depot to ensure path continuity, drone cannot visit a different replicated depot in the middle of the route
    depots_orig_list = []
    for i in range(num_depots):
        depots_orig_list.append(list(range(num_drones * i, num_drones * i + num_drones)))
    depots_orig_list_concat = [item for sublist in depots_orig_list for item in sublist]

    depots_dest_list = []
    for i in range(num_depots):
        depots_dest_list.append(list(range(num_depots_repl + 2 * num_tasks + num_stops * num_tasks + num_drones * i, 
                                        num_depots_repl + 2 * num_tasks + num_stops * num_tasks + num_drones * i + num_drones)))
    depots_dest_list_concat = [item for sublist in depots_dest_list for item in sublist]

    pickup_list = [*range(num_depots_repl, num_depots_repl + num_tasks)]
    delivery_list = [*range(num_depots_repl + num_tasks, num_depots_repl + 2 * num_tasks)]
    bus_stop_list = []
    for i in range(num_stops):
        bus_stop_list.append(list(range(num_depots_repl + 2 * num_tasks + i * num_tasks, num_depots_repl + 2 * num_tasks + i * num_tasks + num_tasks)))

    return (nodes_list, num_depots_repl, depots_orig_list, depots_orig_list_concat, depots_dest_list, depots_dest_list_concat, 
            pickup_list, delivery_list, bus_stop_list)




# Create arcs between nodes that are feasible (see notebook after 20.11)

def create_all_arcs(num_depots_repl, bus_stop_list, pickup_list, delivery_list, 
                    depots_dest_list_concat, num_tasks, bus_travel_times):

    arcs = []

    # x_O_Np: from replicated origin depot to all pickup
    for d in range(num_depots_repl):
        for p in pickup_list:
            arcs.append((d, p))

    # x_O_T: from origin depot to all bus stops
    for d in range(num_depots_repl):
        for s in bus_stop_list:
            for b in s:
                arcs.append((d, b))

    # x_Np_Nd: from pickup to delivery for the same parcel, corresponds to direct drone flight
    for p, d in zip(pickup_list, delivery_list):
        arcs.append((p, d))

    # x_Np_T: from pickup to replicated bus stop related to the same parcel 
    for p_i, p in enumerate(pickup_list):
        for s in bus_stop_list:
            arcs.append((p, s[p_i]))

    # x_Nd_O: from delivery to destination depot
    for d in delivery_list:
        for o in depots_dest_list_concat:
            arcs.append((d, o))

    # x_Nd_Np: from delivery to pickup, all except the corresponding pickup
    for d in delivery_list:
        for p in pickup_list:
            if d != p + num_tasks:
                arcs.append((d, p))

    # x_Nd_T: from delivery to replicated bus stop for different parcel
    for d_i, d in enumerate(delivery_list):
        for s in bus_stop_list:
            for b_i, b in enumerate(s):
                if b_i != d_i:
                    arcs.append((d, b))

    # x_T_O: from bus stop to destination depot
    for s in bus_stop_list:
        for b in s:
            for o in depots_dest_list_concat:
                arcs.append((b, o))

    # x_T_Np: from bus stop to pickup, all except the corresponding pickup
    for s in bus_stop_list:
        for b_i, b in enumerate(s):
            for p_i, p in enumerate(pickup_list):
                if b_i != p_i:
                    arcs.append((b, p))

    # x_T_Nd: from bus stop to delivery, only to the corresponding task
    for s in bus_stop_list:
        for d_i, d in enumerate(delivery_list):
            arcs.append((s[d_i], d))

    # x_T_T: from bus stop to another bus stop, only connect ajacent bus stops replicated for the same parcel
    for (s1, s2) in bus_travel_times: # both directions
        for task in range(num_tasks):
            arcs.append((bus_stop_list[s1][task], bus_stop_list[s2][task]))
    
    return arcs


def create_parcel_arcs(scenario, pickup_list, delivery_list, bus_stop_list, dist_drone_arcs, num_tasks):
    # Parcel - Bus stop matching
    # Option 1: Direct flights only, Np to Nd. No bus. To test the model.
    # Option 2: Np to nearest bus stop (predefine with arcs), Nd to nearest bus stop (predefine with arcs). No direct flights.
    # Option 3: Either nearest bus stop or direct flight; choose the option for given OF. 

    parcel_arcs = []

    if scenario=='directPD':
        # Option 1
        for p, d in zip(pickup_list, delivery_list):
            parcel_arcs.append((p, d))

    elif scenario=='PD-SL':
        # Option 2
        # find nearest bus stop for each Np and Nd, and append related arcs to parcel_arcs
        # for i, (p, d) in enumerate(zip(pickup_list, delivery_list)):
        #     stops_list_for_request = [stop[i] for stop in bus_stop_list]

        #     # find nearest bus stop to pickup
        #     p_stop_node = stops_list_for_request[np.argmin(dist_drone_arcs[p, stops_list_for_request])]
        #     # append replicated bus stop for this request
        #     parcel_arcs.append((p, p_stop_node))

        #     # find nearest bus stop to delivery
        #     d_stop_node = stops_list_for_request[np.argmin(dist_drone_arcs[d, stops_list_for_request])]
        #     # append replicated bus stop for this request
        #     parcel_arcs.append((d_stop_node, d))

        #     # path from pick up stop and delivery stop
        #     # Attention: arcs must be in the order of visits so that we can create scheduling constraints

        #     if p_stop_node == d_stop_node:
        #         pass # should remove the bus arcs and create direct drone flight arcs instead
        #     elif p_stop_node < d_stop_node:
        #         first_stop = p_stop_node
        #         second_stop = first_stop + num_tasks
        #         while second_stop <= d_stop_node:
        #             parcel_arcs.append((first_stop, second_stop))
        #             first_stop = second_stop
        #             second_stop += num_tasks
        #     else: # p_stop_node > d_stop_node
        #         first_stop = p_stop_node
        #         second_stop = first_stop - num_tasks
        #         while second_stop >= d_stop_node:
        #             parcel_arcs.append((first_stop, second_stop))
        #             first_stop = second_stop
        #             second_stop -= num_tasks
            
        # Option 3
        # find nearest bus stop for each Np and Nd, and append related arcs to parcel_arcs
        for i, (p, d) in enumerate(zip(pickup_list, delivery_list)):
            stops_list_for_request = [stop[i] for stop in bus_stop_list]

            # TODO make sure bus stops are on the same line, if more than one line

            # find nearest bus stop to pickup
            p_stop_node = stops_list_for_request[np.argmin(dist_drone_arcs[p, stops_list_for_request])]
            p_stop_distance = dist_drone_arcs[p, p_stop_node]

            # find nearest bus stop to delivery
            d_stop_node = stops_list_for_request[np.argmin(dist_drone_arcs[d, stops_list_for_request])]
            d_stop_distance = dist_drone_arcs[d, d_stop_node]

            p_d_direct_distance = dist_drone_arcs[p, d]

            if p_d_direct_distance < p_stop_distance + d_stop_distance:
                # append P-D direct arc for this request
                parcel_arcs.append((p, d))
            else:
                # append replicated bus stop for this request
                parcel_arcs.append((p, p_stop_node))
                parcel_arcs.append((d_stop_node, d))

                # path from pick up stop and delivery stop
                # Attention: arcs must be in the order of visits so that we can create scheduling constraints
                if p_stop_node < d_stop_node:
                    first_stop = p_stop_node
                    second_stop = first_stop + num_tasks
                    while second_stop <= d_stop_node:
                        parcel_arcs.append((first_stop, second_stop))
                        first_stop = second_stop
                        second_stop += num_tasks
                else: # p_stop_node > d_stop_node
                    first_stop = p_stop_node
                    second_stop = first_stop - num_tasks
                    while second_stop >= d_stop_node:
                        parcel_arcs.append((first_stop, second_stop))
                        first_stop = second_stop
                        second_stop -= num_tasks

    return parcel_arcs



def plot_all_nodes(stop_df, task_df, depot_df, nodes_coords_dict):

    markersize = 200
    num_stops = len(stop_df)

    # Bus stops and lines
    for i, s in stop_df.iterrows():
        plt.scatter(s['stop_x'], s['stop_y'], s=30, marker='s', c='grey')
        if i < num_stops - 1:
            plt.plot([stop_df.iloc[i]['stop_x'], stop_df.iloc[i+1]['stop_x']], 
                    [stop_df.iloc[i]['stop_y'], stop_df.iloc[i+1]['stop_y']], c='grey', linewidth=4)

    plt.scatter([], [], s=50, marker='s', label='Bus stops', c='grey')

    # Parcel pick up and delivery
    for task_ID, t in task_df.iterrows():
        plt.scatter(t['pickup_x'], t['pickup_y'], s=markersize, marker='^', edgecolors='k', facecolors='k')
        plt.annotate(task_ID, (t['pickup_x']-0.1, t['pickup_y']-0.13), fontsize=8, color='white', fontweight='black', fontname='Verdana')

        plt.scatter(t['delivery_x'], t['delivery_y'], s=markersize, marker='o', edgecolors='k', facecolors='k')
        plt.annotate(task_ID, (t['delivery_x']-0.1, t['delivery_y']-0.13), fontsize=8, color='white', fontweight='black', fontname='Verdana')


    plt.scatter([], [], s=30, marker='^', label='Pick up', edgecolors='k', facecolors='k')
    plt.scatter([], [], s=30, marker='o', label='Delivery', edgecolors='k', facecolors='k')


    # Depot
    for depot_ID, d in depot_df.iterrows():
        plt.scatter(d['depot_x'], d['depot_y'], s=markersize, marker='*', edgecolors='k', facecolors='k')
        # plt.annotate(depot_ID, (d['depot_x']-0.05, d['depot_y']-0.05), fontsize=10, color='k')

    plt.scatter([], [], s=100, marker='*', label='Depot', edgecolors='k', facecolors='k')

    # plt.plot([], [], c='b', label='Best path')
    # plt.legend(loc="lower right")

    # Set x and y boundaries
    x_list = []
    y_list = []
    for val in nodes_coords_dict.values():
        x_list.append(val[0])
        y_list.append(val[1])

    lim_tol_x = 0.15 * (max(x_list) - min(x_list))
    plt.xlim(np.floor(min(x_list) - lim_tol_x), np.ceil(max(x_list) + lim_tol_x))

    lim_tol_y = 0.15 * (max(y_list) - min(y_list))
    plt.ylim(np.floor(min(y_list) - lim_tol_y), np.ceil(max(y_list) + lim_tol_y))

    return
