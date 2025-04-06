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

def create_parcel_arcs_weighted_with_tchain(
    pickup_list,
    delivery_list,
    bus_stop_list,
    dist_drone_arcs,
    time_bus_arcs,
    weights,
    max_top_stations=3
):
    """
    对每个包裹, 先计算直接飞行的加权代价, 
    再在最近 max_top_stations 个巴士站里做组合, 
    如果巴士方案更优, 则把 p->p_stop, (p_stop->...->d_stop) 及 (d_stop->d) 
    都加入 parcel_arcs. 中间站间的 T->T 弧用 'while second_stop <= ...' 的方式生成.
    
    Parameters
    ----------
    pickup_list : list of node IDs for pickups
    delivery_list : list of node IDs for deliveries
    bus_stop_list : list of lists, bus_stop_list[s][i] = node ID 
                    (the s-th bus stop for parcel i)
    dist_drone_arcs : 2D array, dist_drone_arcs[i, j] = drone flight distance
    time_bus_arcs : 2D array, time_bus_arcs[i, j] = bus travel time (not used in this snippet but you can adapt)
    weights : dict, e.g. {'drone_dist': 1.0, 'bus_time': 0.2, 'wait_time': 0.5}
    max_top_stations : int, default=3. How many nearest bus stops to consider
    
    Returns
    -------
    parcel_arcs : list of (i, j). The arcs chosen for each parcel's path.
    """

    parcel_arcs = []
    num_tasks = len(pickup_list)

    def estimate_wait_time():
        # 你可以改成真实的时刻表等待计算
        return 5.0

    for idx, (p, d) in enumerate(zip(pickup_list, delivery_list)):

        # --- 1) 计算直接飞行加权代价
        direct_dist = dist_drone_arcs[p, d]
        cost_direct = (weights.get('drone_dist', 1.0) * direct_dist)
        # 如果你需要别的时间或等待, 可以加

        # --- 2) 找 pickup 点离它最近的 top K 个 bus stops
        # bus_stop_list 形如: bus_stop_list[s][i], s=站索引, i=包裹索引
        # 先把该包裹 i 对应的所有站点都拿出来
        stops_for_this_parcel = [one_line[idx] for one_line in bus_stop_list]
        # 计算 p->stop 的距离
        dist_list_p = []
        for stop_node in stops_for_this_parcel:
            dist_p_stop = dist_drone_arcs[p, stop_node]
            dist_list_p.append((stop_node, dist_p_stop))
        dist_list_p.sort(key=lambda x: x[1])
        top_p_stations = dist_list_p[:max_top_stations]

        # 同理, d->stop 的距离
        dist_list_d = []
        for stop_node in stops_for_this_parcel:
            dist_d_stop = dist_drone_arcs[d, stop_node]
            dist_list_d.append((stop_node, dist_d_stop))
        dist_list_d.sort(key=lambda x: x[1])
        top_d_stations = dist_list_d[:max_top_stations]

        best_bus_cost = float('inf')
        best_bus_arcs = []  # 存储T->T路径在内的完整线路

        # --- 3) 遍历 pickup最近 top_p_stations x delivery最近 top_d_stations
        for (p_stop_node, p_stop_dist) in top_p_stations:
            for (d_stop_node, d_stop_dist) in top_d_stations:
                # 先计算无人机飞到 p_stop_node + d_stop_node飞回 delivery
                total_drone_dist = p_stop_dist + d_stop_dist

                # 公交段(如果只是一条arc, 直接 time_bus_arcs[p_stop_node, d_stop_node] 即可;
                # 但你有 chain, 需要在 "p_stop_node -> ... -> d_stop_node" 一串T->T arcs)

                # 简化: Bus行程看做 time_bus_arcs[p_stop_node, d_stop_node].
                # 你若要更精确, 可以在下面 while 里把 time_bus_arcs[first_stop, second_stop] 累加.
                bus_travel_time = 0.0  # 或者先设0, 然后在 while 里一点点加
                # WAITING
                wait_time = estimate_wait_time()

                # 生成所有 T->T 弧 并统计 bus_travel_time
                # (复制你之前的 snippet, 但放在这里只是为了记录 arcs. 
                #  如果你还想算 bus_travel_time 的总和, 也可在循环中累计 time_bus_arcs[first_stop, second_stop].)
                # 先建一个临时 list, 待会儿一起放进 best_bus_arcs
                potential_tchain_arcs = []
                
                if p_stop_node < d_stop_node:
                    first_stop = p_stop_node
                    second_stop = first_stop + num_tasks
                    # 这里累加 bus_travel_time, if needed
                    while second_stop <= d_stop_node:
                        potential_tchain_arcs.append((first_stop, second_stop))
                        # bus_travel_time += time_bus_arcs[first_stop, second_stop] #如需累加的话
                        first_stop = second_stop
                        second_stop += num_tasks
                else:
                    first_stop = p_stop_node
                    second_stop = first_stop - num_tasks
                    while second_stop >= d_stop_node:
                        potential_tchain_arcs.append((first_stop, second_stop))
                        # bus_travel_time += time_bus_arcs[first_stop, second_stop]
                        first_stop = second_stop
                        second_stop -= num_tasks

                # 现在 potential_tchain_arcs 里就是 p_stop_node -> ... -> d_stop_node 的整段 T->T
                # bus_travel_time 你可以在上面的 while中累加, 这里暂时简单写=0
                # ...
                # 这里决定 bus_travel_time = something

                # 加权代价
                cost_combo = (weights.get('drone_dist', 1.0)*total_drone_dist
                              + weights.get('bus_time', 0.0)*bus_travel_time
                              + weights.get('wait_time', 0.0)*wait_time
                              # 如果还要加 "公共汽车运行时间" 也可以
                             )

                if cost_combo < best_bus_cost:
                    best_bus_cost = cost_combo
                    # 构造这条"完整路线"的 arc 集合: p->p_stop, T->T chain, d_stop->d
                    new_arcs = [(p, p_stop_node)]  # p->p_stop
                    new_arcs.extend(potential_tchain_arcs)  # T->T
                    new_arcs.append((d_stop_node, d))       # d_stop->d
                    best_bus_arcs = new_arcs

        # --- 4) 跟 direct flight 比较
        if cost_direct <= best_bus_cost:
            # 直接飞行更优
            parcel_arcs.append((p, d))
        else:
            # 用公交
            for arc in best_bus_arcs:
                parcel_arcs.append(arc)

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
