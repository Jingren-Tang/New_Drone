import gurobipy as gp
from gurobipy import GRB

from preprocess_functions import *


def set_up_grb_model(model, obj, cost_drone_per_km, cost_drone_per_minute,
                     parcel_arcs, arcs, num_nodes, big_M, num_drones, 
                     nodes_list, pickup_list, delivery_list, 
                     depots_orig_list, depots_orig_list_concat,
                     depots_dest_list, depots_dest_list_concat,
                     dist_drone_arcs, v_drone, dist_drone_dict, 
                     bus_travel_times, bus_stop_list, t_bus_dwell, time_bus_arcs
                     ):

    # Create variables
    # Predefined parcel arcs. One path for each parcel. Parcel path created in preprocessing steps. 
    x_parcel_arc = model.addVars(parcel_arcs, vtype=GRB.BINARY, name="x_parcel_arc")
    x_drone_arc = model.addVars(arcs, vtype=GRB.BINARY, name="x_drone_arc") # arcs: all arcs that MAY be used by drones

    # x_..._node = 1 If object departs from a node
    x_parcel_node = model.addVars(num_nodes, vtype=GRB.BINARY, name="x_parcel_node")
    x_drone_node = model.addVars(num_nodes, vtype=GRB.BINARY, name="x_drone_node")

    # Departure time from each node, parcel and drone
    beta_drone = model.addVars(num_nodes, vtype=GRB.CONTINUOUS, lb=0, ub=big_M, name="beta_drone") 
    beta_parcel = model.addVars(num_nodes, vtype=GRB.CONTINUOUS, lb=0, ub=big_M, name="beta_parcel") 

    # Arrival time at each node. Added to define drone subtour constraints
    alpha_drone = model.addVars(num_nodes, vtype=GRB.CONTINUOUS, lb=0, ub=big_M, name="alpha_drone")
    alpha_parcel = model.addVars(num_nodes, vtype=GRB.CONTINUOUS, lb=0, ub=big_M, name="alpha_parcel")

    makespan = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="makespan", ub=big_M)

    # Constraints
    # NODE PRECEDENCE CONSTRAINTS
    # Start with parcel paths, since drones and buses are there to serve parcels

    # Parcels are all served exactly once following the predefined paths
    model.addConstrs((x_parcel_arc[i, j]==1 for i,j in parcel_arcs), name='c_parcel_served')

    # Parcel and drone nodes exist if an arc departs from it at beta[node]>=0. doesnt necessarily mean alpha[node]>0
    model.addConstrs((x_parcel_node[i] == x_parcel_arc.sum(i, "*") for i in nodes_list), name="c_parcel_node")
    model.addConstrs((x_drone_node[i] == x_drone_arc.sum(i, "*") for i in nodes_list), name="c_drone_node")

    # Each drone starts and ends at the same depot (done in c_flow_conservation) AND is deployed exactly once
    for drone_i in range(num_drones):
        same_drone_depot_orig = [depot[drone_i] for depot in depots_orig_list]
        # A drone can only be deployed once
        model.addConstr(gp.quicksum(x_drone_node[i] for i in same_drone_depot_orig)==1, name=f"c_drone_deployment_{drone_i}")


    # SCHEDULING CONSTRAINTS
    # Departure time at each node = 0 if there is no outgoing arc
    model.addConstrs((beta_drone[i] <= x_drone_node[i] * big_M for i in nodes_list), name="c_beta_drone")
    model.addConstrs((beta_parcel[i] <= x_parcel_node[i] * big_M for i in nodes_list), name="c_beta_parcel")
    model.addConstrs((alpha_drone[i] <= x_drone_node[i] * big_M for i in (set(nodes_list) - set(depots_dest_list_concat)) ), name="c_alpha_drone")
    # remove below because it will be bounded by alpha and beta_drone
    # model.addConstrs((alpha_parcel[i] <= x_parcel_node[i] * big_M for i in nodes_list), name="c_alpha_parcel")

    # [Drone scheduling]

    # For each node i except destination depot (alpha=0), drone departure from i after (>=) arrival at i
    model.addConstrs((beta_drone[i] >= alpha_drone[i] for i in (set(nodes_list) - set(depots_dest_list_concat))), name="c_drone_sche1_dep_after_arr")
    model.addConstrs((beta_drone[i] == 0 for i in depots_dest_list_concat), name="c_drone_sche2_depot_dest_beta_0")
    model.addConstrs((alpha_drone[i] == 0 for i in depots_orig_list_concat), name="c_drone_sche3_depot_orig_alpha_0")

    # If drone arc exists: Arrival at node j (>)= departure from node i + travel time
    for (i, j) in arcs:
        model.addGenConstrIndicator(x_drone_arc[i, j], True, 
            alpha_drone[j] == beta_drone[i] + dist_drone_arcs[i, j] / v_drone,
            name=f"c_drone_sche4_fly_time_{i}_{j}")


    # [Parcel scheduling]

    # Parcel arrival time at pick up node == 0; parcel departure time at delivery node == 0
    model.addConstrs((alpha_parcel[p] == 0 for p in pickup_list), name='c_parcel_sche1_pickup_arr')
    model.addConstrs((beta_parcel[d] == 0 for d in delivery_list), name='c_parcel_sche2_delivery_dep')

    # For all visited nodes (pickup + bus): parcel departure from i after (>=) arrival at i
    # similar to flow conservation, but for time
    model.addConstrs((beta_parcel[i] >= alpha_parcel[i] for i in (set(nodes_list) - set(depots_orig_list_concat) - set(depots_dest_list_concat) - set(delivery_list))), 
                    name="c_parcel_time_precedence2_dep_after_arr")

    # Assume x_parcel_arc=1 if in parcel_arcs, ie. all parcels are served; best arcs were preprocessed so there is one path per parcel
    for i, j in parcel_arcs: 
        # 4 possibilities of arcs: P-D, P-T, T-D, T-T
        if i in pickup_list or j in delivery_list:
            # if there is an active x_parcel_arc from Np_i OR active x_parcel_arc to Nd_i
            # drone should fly this arc, as well as an arc into Np_i OR out of Nd_i (enforced by c_flow_conservation)
            model.addConstr(x_drone_arc[i, j] == x_parcel_arc[i, j], name=f'c_drone_parcel_arc_match_{i}_{j}')
            # travel time enforced in c_drone_sche3_fly_time. just need to synchronize parcel time with drone time

            # [node i] alpha_drone [i] <= beta_parcel [i] == beta_drone [i]
            model.addConstr(beta_parcel[i] >= alpha_drone[i], name=f"c_parcel_dep_i_time_lb_{i}")
            model.addConstr(beta_parcel[i] == beta_drone[i], name=f"c_parcel_dep_i_time_ub_{i}")
            
            # [node j] alpha_drone [j] == alpha_parcel [j] <= beta_drone [j]
            model.addConstr(alpha_parcel[j] == alpha_drone[j], name=f"c_parcel_arr_j_time_lb_{j}")
            model.addConstr(alpha_parcel[j] <= beta_drone[j], name=f"c_parcel_arr_j_time_ub_{j}") 

            if i in pickup_list and j in delivery_list: # P-D
                # set all bus arcs in x_drone_arc for this parcel to inactive [speed up]

                task_ID = pickup_list.index(i)
                
                for (s1, s2) in bus_travel_times: # both directions
                    bus_node_i, bus_node_j = bus_stop_list[s1][task_ID], bus_stop_list[s2][task_ID]
                    model.addConstr(x_drone_arc[bus_node_i, bus_node_j] == 0, name=f'c_no_drone_at_stops_if_parcelPD_{bus_node_i}_{bus_node_j}')
                    
            elif i in pickup_list: # P-T
                # arrival time already constrained by drone fly time
                model.addConstr(beta_drone[j] == beta_parcel[j] , name=f"c_parcel_dep_T_ub_PT_{i}")
                
            else: # T-D
                model.addConstr(alpha_drone[i] <= alpha_parcel[i], name=f"c_drone_arr_T_before_parcelarr_{i}")
                model.addConstr(beta_parcel[i] <= alpha_parcel[i] + t_bus_dwell, name=f"c_parcel_dep_T_after_parcelarr_{i}")
                model.addConstr(beta_drone[i] == beta_parcel[i], name=f"c_drone_dep_T_after_parceldep_{i}")

        else: # T-T
            # parcel moves along bus line with bus schedule. only need to define j departure time in P-T arc
            model.addConstr(alpha_parcel[j] == time_bus_arcs[i, j] + beta_parcel[i], name=f"c_parcel_arr_j_TT_{i}_{j}_lb")
            model.addConstr(beta_parcel[j] <= alpha_parcel[j] + t_bus_dwell, name=f"c_parcel_dep_j_TT_{i}_{j}_ub")
            # parcel departure time j from TT already constrained by c_parcel_time_precedence2_dep_after_arr

            # make sure that the same drone does not fly along the bus just to delivery parcel
            model.addConstr(x_drone_arc[i, j] == 0, name=f'c_drone_doesnt_fly_bus_{i}_{j}')

    # DRONE ARC FLOW CONSERVATION
    # [all nodes except depots: in degree = out degree]
    model.addConstrs((x_drone_arc.sum(i, "*") == x_drone_arc.sum("*", i) for i in (set(nodes_list) 
                                                                                - set(depots_orig_list_concat) 
                                                                                - set(depots_dest_list_concat))), 
                                                                                name="c_flow_conservation")
    
    
    # If node is not related to parcel path or depot: drone should not visit
    model.addConstrs((x_drone_node[i] <= (x_parcel_arc.sum(i, "*") + x_parcel_arc.sum("*", i))
                    for i in (set(nodes_list) - set(depots_orig_list_concat) - set(depots_dest_list_concat))), name="c_arc_outgoing")



    # [depot nodes]
    # Drone does not enter origin depot
    model.addConstrs((x_drone_arc.sum("*", i) == 0 for i in depots_orig_list_concat), name="c_drone_no_enter_orig_depot")
    # Drone does not leave destination depot
    model.addConstrs((x_drone_arc.sum(i, "*") == 0 for i in depots_dest_list_concat), name="c_drone_no_leave_dest_depot")
    # Drone leaves from and return to the same depot # TODO need to break symmetry, ok for now
    for i_orig, i_dest in zip(depots_orig_list, depots_dest_list):
        model.addConstrs((x_drone_arc.sum(i_orig, "*") == x_drone_arc.sum("*", i_dest) for i_orig, i_dest in zip(i_orig, i_dest)), name="c_drone_return_to_depot")


    # MAKESPAN: time that the last drone returns to depot
    model.addConstrs((makespan >= alpha_drone[j] for j in depots_dest_list_concat), name="c_makespan")

    # ADDITIONAL CONSTRAINTS TO SPEED UP OPTIMIZATION

    # Run one round of optimization and when stuck, use upper bound as big_M

    # Parcel pickup to delivery: departure from pick up node + drone travel time <= arrival at delivery node
    for i, j in zip(pickup_list, delivery_list):
        model.addConstr(beta_parcel[i] + dist_drone_arcs[i, j]/v_drone <= alpha_parcel[j], name=f'c_parcel_P_before_D_{i}_{j}')
        model.addConstr(beta_drone[i] + dist_drone_arcs[i, j]/v_drone <= alpha_drone[j], name=f'c_drone_P_before_D_{i}_{j}')

    if obj==1:
        # Option 1: minimize total drone travel distance
        model.setObjective(x_drone_arc.prod(dist_drone_dict), GRB.MINIMIZE)
    elif obj==2:
        # Option 2: minimize makespan (time that the last drone returns to depot)
        model.setObjective(makespan, GRB.MINIMIZE)
    elif obj==3:
        # Option 3: minimize operational cost
        model.setObjective(x_drone_arc.prod(dist_drone_dict) * cost_drone_per_km 
                        +makespan * cost_drone_per_minute,                
                        GRB.MINIMIZE)

    # Option 4: maximize revenue: add drone flight cost, parcel revenue


    return model, x_drone_arc, makespan, beta_drone, alpha_drone, beta_parcel, alpha_parcel, x_parcel_arc


def store_drone_arcs_soln(depots_orig_list, num_drones, x_drone_arc, 
                          depots_orig_list_concat, depots_dest_list_concat,
                          pickup_list, delivery_list, bus_stop_list, beta_drone=None, alpha_drone=None):


    # store the path of each drone in a list
    drone_arcs_solution = []

    for drone_ID in range(num_drones):
        drone_arcs_solution.append([])

        depot_nodes = [depot[drone_ID] for depot in depots_orig_list]

        # Frist arc from depot origin
        # arc solutions from main and subp are of different object types
        if isinstance(x_drone_arc, np.ndarray): # main prob
            next_arc = [(i,j) for i in depot_nodes for j in range(len(x_drone_arc)) if x_drone_arc[i,j]>=0.5][0]
        else: # subp, x_drone_arc is gurobi variable
            next_arc = [(i, j) for (i, j) in x_drone_arc.keys() if x_drone_arc[i,j].Xn>=0.5 and i in depot_nodes][0]
        # TODO fix this if more than 1 depot


        node_i, node_j = next_arc
        depot_i = node_i
        # the corresponding depot dest ID, but for now, it might also be another one due to symmetry
        depot_dest_i = depots_dest_list_concat[depots_orig_list_concat.index(depot_i)]

        # first arc in path
        drone_arcs_solution[-1].append(next_arc)

        # find the position of node_i in depot_nodes list, makes a difference if there are multiple depots
        depot_ID = depot_nodes.index(depot_i)

        if beta_drone is not None:
            # departure time from depot
            beta_i = round(beta_drone[depot_i].Xn, 3)
            print(f'Drone {drone_ID}: Depot {depot_ID} ({beta_i}) -> ', end='') # print next entry in the same line
        else:
            print(f'Drone {drone_ID}: Depot {depot_ID} -> ', end='')

        while node_j!= depot_dest_i: 
        # while node_j not in depots_dest_list_concat:
            if beta_drone is not None:
                # time of departure from next node in path
                beta_j = round(beta_drone[node_j].Xn, 3)
                
                # check if node is a pickup, delivery, or bus stop
                if node_j in depots_orig_list_concat:
                    print(f'DIFF ORIG DEPOT ({beta_j})', end='')
                
                elif node_j in depots_dest_list_concat:
                    print(f'DIFF DEST DEPOT ({beta_j}) ', end='')
                    break
                
                elif node_j in pickup_list:
                    print(f'Pickup {pickup_list.index(node_j)} ({beta_j}) -> ', end='')

                elif node_j in delivery_list:
                    print(f'Delivery {delivery_list.index(node_j)} ({beta_j}) -> ', end='')

                else:
                    for i, s in enumerate(bus_stop_list):
                        if node_j in s:
                            print(f'Bus stop {i} ({beta_j}) -> ', end='')
                            break
            else: # beta_drone is None
                # check if node is a pickup, delivery, or bus stop
                if node_j in depots_orig_list_concat:
                    print(f'Problem. DIFF ORIG DEPOT ', end='')
                
                elif node_j in depots_dest_list_concat:
                    print(f'DIFF DEST DEPOT ', end='')
                    break
                
                elif node_j in pickup_list:
                    print(f'Pickup {pickup_list.index(node_j)} -> ', end='')

                elif node_j in delivery_list:
                    print(f'Delivery {delivery_list.index(node_j)} -> ', end='')

                else: # bus stop
                    for i, s in enumerate(bus_stop_list):
                        if node_j in s:
                            print(f'Bus stop {i} -> ', end='')
                            break

            # Above: identified where the current arc goes (node_j).
            # Below: check where node_j leads to
            if isinstance(x_drone_arc, np.ndarray): # Main prob            
                next_node_j = np.where(x_drone_arc[node_j]>=0.5)[0][0]
            else: # subp, x_drone_arc is gurobi variable
                next_node_j = [j for (i, j) in x_drone_arc.keys() if x_drone_arc[i,j].Xn>=0.5 and i==node_j][0]


            node_i, node_j = node_j, next_node_j
            next_arc = (node_i, node_j)
            drone_arcs_solution[-1].append(next_arc)

        if beta_drone is not None:
            print(f'Depot {depot_ID} ({round(alpha_drone[depot_dest_i].Xn, 3)})')
        else:
            print(f'Depot {depot_ID}')


    return drone_arcs_solution

def plot_drone_arcs_soln(drone_arcs_solution, num_drones, nodes_coords_dict, dist_drone_arcs):

    drone_dists = np.zeros((num_drones,))
    for drone_ID, drone_path in enumerate(drone_arcs_solution):
        for i, j in drone_path:
            plt.plot([nodes_coords_dict[i][0], nodes_coords_dict[j][0]],
                    [nodes_coords_dict[i][1], nodes_coords_dict[j][1]], 
                    c=['blue', 'pink', 'gold'][drone_ID], alpha=0.5)
            
            drone_dists[drone_ID] += dist_drone_arcs[i, j]
        plt.plot([], [], c=['blue', 'pink', 'gold'][drone_ID], alpha=1, label=f'Drone {drone_ID}')

    plt.legend()
    plt.xlabel('x-coordinate (km)')
    plt.ylabel('y-coordinate (km)')

    return