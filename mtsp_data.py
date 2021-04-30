import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from datetime import datetime
from scipy.spatial import distance_matrix
import random
import matplotlib.pyplot as plt
import numpy as np

#Comment for Remote Machine
# import matplotlib
# matplotlib.use('Agg') #Using Non-interactive backend

num_uav = 3
num_ugv = 1
grid_size = 3


depot = [0,0]


class Task:
    def __init__(self, identity, location):
        self.identity = identity
        self.location = location
        self.params = params # Quantification of the task

class Agent:
    def __init__(self, identity, location):
        self.identity = identity
        self.location = location
        self.status = status # Free/Operational/Failed

class Event:
    def __init__(self, name, location):  
        self.name = name
        self.location = location


def event_handler(event, locations, num_vehicles):
    if event=="New":
        new_x = int(input("Enter X Co-ordinate: "))
        new_y = int(input("Enter Y Co-ordinate: "))
        if (new_x!=0 and new_y!=0):
            # locations = data['locations']
            locations.append([new_x, new_y])
            # data['locations'] = locations
            # print(len(data['locations']))
    if event=="Failed":
        num_vehicles = int(input("Number of Operational Vehicles: "))
        # data['num_vehicles'] = num_vehicles

    return locations, num_vehicles

def location_handler():
    # locations = [[random.randint(-grid_size,grid_size) for i in range(2)] for j in range(num_loc + 1)]
    locations = []
    
    i_coords, j_coords = np.meshgrid(range(grid_size), range(grid_size), indexing='ij')
    coordinate_grid = np.array([i_coords, j_coords])
    for i in range(grid_size):
        for j in range(grid_size):
            loc_x = 8*coordinate_grid[:, i, j][0]
            loc_y = 8*coordinate_grid[:, i, j][1]
            if (loc_x == 0 and loc_y!=0):
                locations.append([loc_x, loc_y])
                locations.append([loc_x, -loc_y])
            elif loc_x!=0 and loc_y==0:
                locations.append([loc_x, loc_y])
                locations.append([-loc_x, loc_y])
            elif loc_x==0 and loc_y==0:
                print("Yay")
                # locations.append([loc_x, loc_y])
            else:
                locations.append([loc_x, loc_y])
                locations.append([-loc_x, loc_y])
                locations.append([loc_x, -loc_y])
                locations.append([-loc_x, -loc_y])

    locations[0] = depot #Depot Location
    return locations

def create_distmat(locations):
    distMat = [ [ 0 for i in range(len(locations)) ] for j in range(len(locations)) ] 
    for i in range(len(locations)):
        for j in range(len(locations)):
            dist = int(math.sqrt((locations[j][0] - locations[i][0])**2 + (locations[j][1] - locations[i][1])**2))
            distMat[i][j] = dist
    return distMat


def create_data_model(locations, num_vehicles):
    """Stores the data for the problem."""
    data = {}
    
    data['num_vehicles'] = num_vehicles
    data['distance_matrix'] = create_distmat(locations)
    data['locations'] = locations
    print(len(data['locations']))
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    route_map = []
    for vehicle_id in range(data['num_vehicles']):
        route_map.append([vehicle_id, 0])
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            route_map.append([vehicle_id, index])
        route_map.pop(-1)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
        route_map.append([vehicle_id, 0])
    print('Maximum of the route distances: {}m'.format(max_route_distance))
    return route_map

def plot_solution(data, route_map):
    #Plotter Variable
    plt.figure()
    colors = ["black", "green", "red", "yellow", "blue", "cyan", "magenta", "olive", "orange", "tomato"]
    plt.grid()
    lim = max(max(data['distance_matrix']))
    plt.xlim(-lim/2, lim/2) #Xlim should be the max of xLoc
    plt.ylim(-lim/2, lim/2) #YLim should be the max of yLoc
    plt.plot(data['locations'][0][0], data['locations'][0][1], 'x')
    plt.text(data['locations'][0][0], data['locations'][0][1], 'Depot', fontsize='10')
    
    for i in range(len(route_map)-1):
        x_values = [data['locations'][route_map[i][1]][0], data['locations'][route_map[i+1][1]][0]]
        y_values = [data['locations'][route_map[i][1]][1], data['locations'][route_map[i+1][1]][1]]
        # print(data['locations'][route_map[i][1]])
        plt.scatter(data['locations'][route_map[i][1]][0], data['locations'][route_map[i][1]][1],  c=colors[route_map[i][0]])
        plt.plot(x_values, y_values, c=colors[route_map[i][0]])
        plt.text(data['locations'][route_map[i][1]][0], data['locations'][route_map[i][1]][1], route_map[i][1], fontsize='10')
        plt.pause(0.05)
    # plt.savefig('plot.png')
    # plt.show()
    # plt.matshow(data['distance_matrix'])
    # plt.colorbar()
    # plt.savefig('dist_mat.png')

def mtsp_solver(data):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    start_time = datetime.now()
    

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    end_time = datetime.now()
    exec_time = end_time-start_time
    print('Execution Time: %s' % (exec_time))
    if solution:
        route_map = print_solution(data, manager, routing, solution)
    return route_map

def main():
    for i in range(3):
        locations = location_handler()
        num_vehicles = num_uav + num_ugv
        event_type = str(input("Type of event: [No/New/Failed] : "))
        locations, num_vehicles = event_handler(event_type, locations, num_vehicles)
        data = create_data_model(locations, num_vehicles)
        route_map = mtsp_solver(data)
        plot_solution(data, route_map)
    
    input("Press [Enter] to Exit!")



if __name__ == '__main__':
    main()
