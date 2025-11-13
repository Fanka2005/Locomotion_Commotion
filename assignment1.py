import math
import heapq
def intercept(roads: list[tuple], stations: list[tuple], start: int, friendStart: int) -> tuple|None:
    """
		Function description: This function determines the least cost path to intercept a moving passenger on a train loop

		Approach description: This function models a road network and a cyclic train system. 
                                It first computes the time taken for a friend on a train loop to pass through each station using total_station_time(). 
                                Then, it constructs a graph representation of the road network with adjacency_list(). 
                                Using a modified Dijkstra's algorithm (modified_dijkstra()), it calculates the least-cost driving path from the user's starting location to all possible location, including the station, over all possible time state. 
                                Finally, optimum_path() selects the best interception point (station and time) where the user can meet the friend with minimum cost and earliest arrival time,
                                from the output of modified_dijkstra().
                                The route, total cost, and total time to that intercept point are then returned.

		Input: 
			roads  (list[tuple]) : a list of roads where each road is represented as a tuple containing  (lu, lv, c, t) where:
                                    lu is the starting location of the road,
                                    lv  is the ending location of the road,
                                    c is the positive integer cost of travelling down the road from lu to lv,
                                    t is the time in minutes as a positive integer required to travel from lu to lv

            stations (list[tuple]): a list of train stations where each station is represented as a tuple (lTS, t) where:
                                    lTS is a train station,
                                    t is the time in minutes to travel from lTS to the next train station in the train loop,
           
            start (int) : is the location from where you start the trip (int)

            friendStart (int): is the train station from where your friend starts their journey
		
		Output: a tuple (totalCost, totalTime, route), where: 
                - totalCost is the total cost of the journey along the optimal valid route. This should be a positive integer.
                - totalTime is the total time driven of the journey along the optimal valid route. This should be a positive integer.
                - route is the route you can use to drive from start to the train station you intercept your friend at. This should be a list of integers in the range of 0 to |L| - 1, that represent the
                    locations you travel along the route, ending with the intercepting train station.
		
		Time complexity: O(|R|log(|L|)), where |R| is the total number of roads and |L| is the total number of location

		Time complexity analysis :Given |R| is the total number of roads and |L| is the total number of location
                                    This function will run multiple function inside, which are : 
                                    :total_station_time() : time complexity : O(1),
                                    :adjacency_list() : time complexity :  O(|R|),
                                    :modified_dijkstra() : time complexity : O(|R|log(|L|)+|L|),
                                    :optimum_path() : time complexity : O(|L|),
                                    Therefore the total time complexity : O(|R|log(|L|)+|R|+2|L|),
                                    Because each location must have at least one outgoing road, therefore O(|L|)<=O(|R|): 
                                    Thus the final time complexity : O(|R|log(|L|))

		Space complexity: O(|R|+|L|), where |R| is the total number of roads and |L| is the total number of location

        Space complexity Analysis: Given |R| is the total number of roads and |L| is the total number of location
                                    This function will run multiple function inside, which are : 
                                    :total_station_time() : space complexity : O(1),
                                    :adjacency_list() : space complexity :  O(|R|),
                                    :modified_dijkstra() : space complexity : O(|R|+|L|),
                                    :optimum_path() : space complexity : O(|L|),
                                    Therefore the total space complexity : O(2|R|+2|L|),
                                    Thus the final space complexity : O(|R|+|L|)
		
	"""

    #Validate input
    #Time and Space Complexity:  O(|T|), because |T| is the number of stations and cannot exceed 20, then
    #Final time and space complexity: O(1)
    if start < 0 or friendStart < 0 or len(roads) == 0 or len(stations) == 0:
        return None
    if friendStart not in [station[0] for station in stations]:
        return None
    
    #This function calculate the amount of time it tooks to travel from the source station/element to each station/element and back to itself.
    #The time and space for total_station_time(): O(|T|), where |T| is the number of station
    #because the number of station will not exceed 20 stations, 
    #Thus the final time and space complexity is O(1)
    #Output :  An array based list containing tuple list[tuple], which the tuple contains (station, the distance to each stations/element from the source station/element)
    station_time_list = total_station_time(stations, friendStart)

    #This function create an adjacency list from the road list input.
    #The time and space for adjacency_list(): O(|R|+|L|), where |R| is the number roads/elements in the input list and |L| is the number of location
    #Because each location must have at least one outgoing road, therefore |L|<=|R|, 
    #Thus the the final time and space complexity is O(|R|)
    #Output : An array based list[list[tuple]], where the outer list index is the locationA, and the inner list contains all location tuple (the location adjacent to locationA, its cost, and time)
    adjacencylist = adjacency_list(roads)

    #This function implements a Dijkstra's algorithm to find the minimum cost of interception path
    #The time complexity : O(|R|log(d*|L|)+|L|*d),
    #The space complexity : O(|R| + |L|*d),
    #where |R| is the number of roads and |L| is the number of location available and worst case d = |T|*(the time between stations), where |T| is the number of station in station_time_list.
    #because the number of station will not exceed 20 and the time taken between station cannot exceed 5 minutes, thus d=20*5=100 and |T|<=20
    #Therefore :
    #   the final time complexity : O(|R|log(|L|)+|L|),
    #   the final space complexity:  O(|R| + |L|),
    dcost_time, path = modified_dijkstra(adjacencylist, start, station_time_list)

    #This function is used to choose the optimum interception path to the station.
    #The time complexity for optimum_path(): O(|T|+d*|L|), 
    #The space complexity for optimum_path(): O(d*|L|), 
    #where |L| is the number of location available and worst case d = |T|*(the time between stations), where |T| is the number of station.
    #because the number of station will not exceed 20 and the time taken between station cannot exceed 5 minutes, thus d=20*5=100 and |T|<=20
    #Therefore :
    #   the final time complexity : O(|L|)
    #   the final space complexity : O(|L|)
    # Ouput : a tuple containing (cost, location ID, time, route)
    result = optimum_path(dcost_time, path, station_time_list, start)

    if result == None :
        return None
  
    return (result[0], result[2], result[-1])

##############################################################################################################################################################

def adjacency_list(roads_list: list[tuple])->list[list[tuple]]:
    """
		Function description: This function create an adjacency list from the road list input.
		Approach description: This function created a new empty list (adjacency_road_list) and a number of sublist inside, for each location, and then iterate over all roads/item in the input list (roads_list).
                                    In each iteration, it input to the sublist in index A the location that the road/edge leads to from locationA, where the index represent, assume locationA,
                                    and then the function inputed some tuple into the sublist that contains (the location adjacent to locationA, its cost, and time).

		Input: 
			roads_list (list[tuple]): an array based list containing tuple with 4 item (locA, locB, cost, time)
		
		Output: An array based list[list[tuple]], where the outer list index is the locationA, and the inner list contains all location tuple (the location adjacent to locationA, its cost, and time)
		
		Time complexity: O(|R|+|L|), where |R| is the number roads/elements in the input list and |L| is the number of location

		Time complexity analysis : Given |R| is the number roads/elements in the input list and |L| is the number of location,
		

			The function create and adjacency_road_list and then iterate from 0 to maximum location ID, appending an empty list into adjacency_road_list in each iteration, 
            This has O(|L|) time complexity.
            Next, it will iterate every item/road in the input list and perform a constant operation for every iteration (apending tuple).  
            This has O(|R|) time complexity.
            thus it is  O(|R|+|L|) time complexity

		Space complexity: O(|R|+|L|),  where |R| is the number roads/elements in the input list and |L| is the number of location


		Space complexity analysis: Given |R| is the number roads/elements in the input list and |L| is the number of location,
                                    this function created an empty list and a number of sublist inside, for each location, and then iterate over all roads/item in the input list.
                                    In each iteration, it input a sublist in each index, where the index represent location, assume locationA,
                                    and then the sublist it inputed is some tuple that contains the location adjacent to locationA, its cost, and time.
                                    Therefore, space complexity : 
                                    : O(|L|) + O(|R|)
		
	"""

    #Iterate the input list and get the maximum location ID
    #Time : O(|R|), Space O(1), where |R| is the number of roads.
    max_loc = max(loc[0] for loc in roads_list)
    
    #Initialise a list containing sublist from 0 to max_loc index
    #Time : O(|L|), Space: O(|L|), where |L| is the number of location.
    adjacency_road_list = []
    for _ in range(max_loc+1): 
        adjacency_road_list.append([])

    #Append a tuple that is adjacent to locationA inside the sublist
    #Time : O(|R|), Space: O(|R|), where |R| is the number of roads.
    for edge in roads_list: 
        adjacency_road_list[edge[0]].append((edge[1], edge[2], edge[3]))

    #Return a tuple (the location adjacent to locationA, its cost, and time).
    return adjacency_road_list


def total_station_time(stations: list[tuple], friendStart: int)->list[tuple]: 
    """
		Function description: This function calculate the amount of time it tooks to travel from the source station/element to each station/element and back to itself.
		Approach description: This function create dist_time list and then iterate over each station/element in input list/stations to find the index of the source station,
                                after that, create a new list by slicing the input list/stations from the source index to the end and from the front to the source index, then concatenate them together.
                                next it iterate the new list while updating the time (It increment the time in every iteration by the time value in every station) 
                                and then append the tuple containing (station, updated distance time from the source station) to the dist_time list

		Input: 
            stations(list[tuple]) : An array based list of train stations where each station is represented as a tuple (lTS, t) where:
                                    - lTS is a train station,
                                    - t is the time in minutes to travel from lTS to the next train station in the train loop,

            friendStart(int) : is the train station ID from where your friend starts their journey 

		Output: An array based list containing tuple list[tuple], which the tuple contains (station, the distance to each stations/element from the source station/element)
		
		Time complexity: O(|T|), where |T| is the number of station/element in input list/stations

		Time complexity analysis : Given |T| is the number of station/element in input list/stations,
		
			The function iterate every item in the input list/stations and perform a constant operation for every iteration (unpack tuple, apending and incrementing).
            This function created some empty list and then iterate over all item in the input list and append a tuple (station, the time it takes) in each iteration.
                                    It increment the time in every iteration by the time value in every station, thus it only takes O(T) time where |T| is the number
                                    of element/station in the input list/stations.
                                    O(|T|) : Iterate the stations/input list to find the source index
                                    O(|T|) : Calculate the amount of time it tooks to travel from the source station to each station, by incrementing and appending to the dist_time list as you go
                                    therefore : O(|T|+|T|) = O(|T|)

		Space complexity: O(|T|),  where |T| is the number of station/element in input list/stations.

		Space complexity analysis: This function created some empty list and then iterate over all item in the input list and append a tuple (station, the time it takes) in each iteration.
                                    It increment the time in every iteration by the time value in every station, thus it only takes O(T) space where |T| is the number
                                    of element/station in the input list/stations.
                                    O(|T|) : input list (stations)
                                    O(|T|) : the new created list by slicing, total
                                    O(|T|) : the new created list (dist_time)
                                    therefore : O(|T|+|T|+|T|) = O(|T|)
		
	"""

    #Find the source index
    #Time and space : O(T), where T is number of train stations
    for index, station in enumerate(stations): 
        if station[0] == friendStart:
            source_index = index

    dist_time=[]
    #Calculate the amount of time it tooks to travel from the source station to each station, by incrementing and appending to the dist_time list as you go
    #Time and space : O(T), where T is number of train stations
    counter = 0
    for index, station in enumerate(stations[source_index:]+stations[:source_index]):
        dist_time.append((station[0], counter))
        counter += station[1]

    #Update the time it took to loop back
    #Time and space : O(1)
    dist_time[0] = (dist_time[0][0], counter)

    return dist_time

def modified_dijkstra(adjacency_list: list[tuple], start : int, station_time_list: int) -> tuple: 
    """
		Function description:
            This function implements a Dijkstra's algorithm to find the minimum cost path 
            from a start location to all other locations, while also considering time-dependent states 
            (modular time) to enable interception based on friend's train station location.

        Approach description:
            In the 2D list (Matrix) of dcost_time and path: where the rows represent the location and the column represent the time state.
            There is 0 to (time it takes for the train to loop back to it source station) time-state/column in the dcost_time and path matrix.
            Each node of dcost_time matrix contain a tuple (total cost to that location, time to that location),
            while each node of path matrix contain a tuple (previous node (rows), the time state (column) of that previous node)
            where 'time_state' represents the (current time % maximum time the train loops back to the source station). 
            The algorithm uses a min-heap to prioritize (cost, time, locationID) tuple that have lower total cost and earlier total time to that location,
            It will then iterate through all adjacent location.
            and it updates paths and costs in dcost_time and path whenever a cheaper or faster route to a time-state is found.
            this function will stop when the min heap priority queue is empty, which means no path have a smaller cost or time.

        Prove of correctnes: 
            This function computes the minimum-cost and earliest-arrival paths from a starting location to all others under modular time constraints. 
            It extends the standard Dijkstra's algorithm by maintaining a 2D matrix dcost_time[node][time_state] and path[node][time_state], where time_state = time % max_time, to account for time-dependent states. 
            At each iteration, the algorithm selects the node with the lowest cost (and earliest time) from a min-heap priority queue, and ensuring outdated paths are skipped. 
            It updates paths only when a cheaper or faster route is found, maintaining optimality. 
            This process continues until all reachable time-states nodes are processed, which guarantees that the final matrices store the correct minimum costs, arrival times, and valid path traces for backtracking.

        Input:
            adjacency_list (list[list[tuple]]): Adjacency list representing the roads.
            start (int): Index/location ID of the starting location.
            station_time_list (list[tuple]): List of (station, time to that station) pairs.

        Output: A tuple containing (dcost_time, path):
                - dcost_time (list[list[tuple]]): A 2D list (Matrix) where each entry (i, t) holds the minimum (cost, total_time) to reach node i at time state :t % max_time.
                - path (list[list[tuple | None]]): A 2D list (Matrix) to trace back the previous node and time state for path reconstruction.

		Time complexity: O(|R|log(d*|L|)+|L|*d), where |R| is the number of roads and |L| is the number of location available and d is the time it takes to loop back to the source station or worst case d= |T|*(the time between stations), where |T| is the number of stations

		Time complexity analysis : Given |R| is the number of roads and |L| is the number of locations and d = |T|*(the time between stations), where |T| is the number of station in station_time_list,
                                    The function will first create two |L|*d matrix, This has O(|L|*d) time complexity

                                    Dijsktra algorithm:
                                    It will then initialise the first (cost, time, locationID) and push it into the min heap priority queue, 
                                    It then checks whether the queue is empty and while the queue is not empty,
                                    it will pop the tuple (cost, time, locationID) with the minimum cost and time.
                                    It will then check whether the tuple is an outdated tuple.
                                    Then it will iterate all location adjacent to that tuple locationID 
                                    In each iteration:
                                        if a cheaper or faster route to a time-state is found, it will updates paths and costs in dcost_time and path
                                        and then create the tuple for the adjacent location and push it into the min heap priority queue.
                                    This will be repeated until the min heap priority queue is empty, thus all the total cost and time in dcost_time and all the nodes in path will be the updated version.

                                    The dijkstra algorithms have O(|R|log(d*|L|)) time complexity, 
                                    because it iterates all possible roads/adjacent location of a particular location (|R|).
                                    if worst case scenario would be that the min heap priority queue length would be d*|L|,
                                    then in each iteration, the time complexity of push or pop of the min heap priority queue is O(log(d*|L|))

                                    Thus the total time complexity is O(|R|log(d*|L|)+|L|*d)
	
		Space complexity: O(|R| + |L|*d), where |R| is the number of roads and |L| is the number of location available and d = |T|*(the time between stations), where |T| is the number of station in station_time_list.
                 
		Space complexity analysis: Given |R| is the number of roads and |L| is the number of locations and d = |T|*(the time between stations), where |T| is the number of station in station_time_list,
                                    The function will have an input of : 
                                    : adjacency_list : O(|R|) space complexity
                            
                                    and then the function will create two |L|*d matrix, This has O(|L|*d) space complexity
                                    It will then uses a min heap priorty queue, which in worst case have approximately O(|L|*d) space complexity. 
                                    because it may stores all updated location.

                                    Therefore the space complexity is O(|R| + |L|*d)
		
	"""

    #Initialise the min heap priority queue container
    #Space complexity worst case : O(|L|*d)
    min_heap = []

    #Get the maximum station time (the time a train takes to loop back to the source station)
    max_time = station_time_list[0][1]

    index_source = start

    #create two |L|*d matrix that will store the path and cost and time to get to that location
    dcost_time = []
    for i in range(len(adjacency_list)):
        dcost_time.append([])
        for j in range(max_time+1):
            dcost_time[i].append((math.inf, math.inf))

    path = []
    for i in range(len(adjacency_list)):
        path.append([])
        for j in range(max_time+1):
            path[i].append(None)
    
    #Initialise the starting location
    dcost_time[index_source][0] = (0, 0)
    dcost = 0
    dtime = 0
   
    #Push the starting location tuple to the min heap priority queue
    #Time : O(log|L|)
    heapq.heappush(min_heap, (dcost, dtime, index_source))

    #Continue until min heap priority queue is empty
    #Time : O(|R|log(d*|L|))
    while len(min_heap)!=0: 
    
        #Extract the node with the minimum cost (and earliest time)
        #Time : O(log|L|)
        current_cost, current_time, node = heapq.heappop(min_heap) 

        #Get the modular time state for the current node (used for synchronization)
        current_time_state = current_time%max_time

        #Will not process an outdated node/tuple
        if dcost_time[node][current_time_state][0] == current_cost and dcost_time[node][current_time_state][1] == current_time:
    
            #Iterate over all adjacent locations (roads) from the current location
            for adjacent_node in adjacency_list[node]: 
            
                index_adjacent_node = adjacent_node[0]

                #Calculate the modular time state of the adjacent node
                next_time_state = (current_time + adjacent_node[-1]) % (max_time)
                
                #Get the currently recorded cost and time to reach the adjacent node at that time state
                dcost = dcost_time[index_adjacent_node][next_time_state][0]
                dtime = dcost_time[index_adjacent_node][next_time_state][1]

                # Compute the new cost and time of this adjacent path 
                new_cost = current_cost + adjacent_node[1]
                new_time = current_time + adjacent_node[-1]

                #Update only if the new path is better in terms of cost,
                #or if cost is the same but new path has an earlier arrival time
                if  dcost > new_cost or (dcost == new_cost and dtime > new_time):

                    #Record the new optimal cost and time to reach this neighbor
                    dcost_time[index_adjacent_node][next_time_state] = (new_cost, new_time)

                    #Track the path for backtracking: store current node/locationID and its time state
                    path[index_adjacent_node][next_time_state] = (node, current_time_state)
                    
                    #Push the updated path info into the min heap priority queue 
                    #Time : O(log|L|)
                    heapq.heappush(min_heap, (new_cost, new_time, index_adjacent_node))

    return (dcost_time, path)


def optimum_path(dcost_time: list[list[tuple]], path: list[list[tuple]], station_time_list: list[tuple], start: int) -> tuple|None:
    """
		Function description: This function is used to choose the optimum interception path to the station.
		Approach description and proof of correctness: 

            Initialization:

                Set optimum_path to an initial value ((inf, 0, 0)) to store the best (minimum cost) station candidate.

                Set target_time to track the time state corresponding to the optimal path.

            Iterate through stations in station_time_list:

                For each (station, time_state), check if the path to this station and time state is reachable (dcost_time[station][time_state] is not None).

                Retrieve the driving cost (dcost) and driving time (dtime).

                Update optimum_path if this station has a lower cost or if the cost is equal but the total time is lower. 

            Check for valid result:

                If no valid interception path is found, return None. 

            Backtrack the path:

                After an optimum interception path is found, Use path_backtrack function to reconstruct the full path from start to the chosen target station and time.

            Therefore, this function will always return the least cost and time interception path

        Input: 
            dcost_time (list[list[tuple]]): A matrix containing tuple (cost, time) for each location and their time state.
            path (list[list[tuple]]): A matrix containing tuple (previous Index, previous Index time state).
            station_time_list (list[tuple]): An array based list containing tuple (station ID, their time from the source station)
            start (int): The driver start location ID

		Output: A tuple containing (cost, station ID, time, reconstructed path from the start to that station)

		Time complexity: O(|T|+d*|L|), where |L| is the number of location available and d is the time it takes to loop back to the source station or worst case d= |T|*(the time between stations), where |T| is the number of stations
		Time complexity analysis : Given |L| is the number  number of location available,
                                    Because this function chooses the optimum interception path, it will iterate over all station in their particular time state,
                                    This has O(|T|) time complexity, where |T| is the number of station.
                                    After that it will reconstructs the path from a target station back to the starting driver location by using path_backtrack function, which has O(d*|L|) Time and Space Complexity,
                                    where d = |T|*(the time between stations) and |T| is the number of station,
                                    Thus, the time complexity : 
                                    : O(|T|) + O(d*|L|) = O(|T|+d*|L|)

                                    Note : time state is the modular time state for each station, meaning that for the driver to intercept their friends at a certain station, assume station A
                                            the driving time between the start location to that station A must : (the driving time between the start location to that station) % (the time needed for the train to loop back) = the time from the friend start station to station A

		Space complexity: O(d*|L|),  where |L| is the number of location available and d is the time it takes to loop back to the source station or worst case d= |T|*(the time between stations), where |T| is the number of station in station_time_list.
		Space complexity analysis: Given |L| is the number  number of location available,
                                    Because this function chooses the optimum interception path, it will iterate over all station in their particular time state, this has O(1) space complexity,
                                    After that it will reconstructs the path from a target station back to the starting driver location by using path_backtrack function, which has O(|L) Time and Space Complexity.
                                    Thus, the space complexity : 
                                    : O(|L|), for the input dcost_time matrix
                                    : O(|L|), for the input path matrix
                                    : O(|L|), for the input station_time_list
                                    : O(1), for the start location ID input
                                    : O(d*|L|), for the space complexity of path_backtrack, where |L| is the number of location available and d = |T|*(the time between stations), where |T| is the number of station,
                                    Therefore the time complexity is O(d*|L|) = O(d*|L|)

                                    Note : time state is the modular time state for each station, meaning that for the driver to intercept their friends at a certain station, assume station A
                                            the driving time between the start location to that station A must : (the driving time between the start location to that station) % (the time needed for the train to loop back) = the time from the friend start station to statioan A

	"""
   
    #Set optimum_path to an initial value ((inf, 0, 0)) to store the best (minimum cost) station candidate.
    optimum_path = (math.inf, 0, 0)

    #Set target_time to track the time state corresponding to the optimal path.
    target_time = 0

    #For each (station, time_state), check if the path to this station and time state is reachable (dcost_time[station][time_state] is not None).
    #Time: O(|L|), Space: O(1), where |L| is the number of location available
    for time in station_time_list:
        station, time_minutes = time
        
        #Update the time minutes to 0 for the start station, this is because the time state of the start station is 0, as (start station time)%max station time = 0
        index_station = station
        if time_minutes == station_time_list[0][1]:
            time_minutes = 0

        #Retrieve the driving cost (dcost) and driving time (dtime).
        if dcost_time[index_station][time_minutes] != None:
            dcost = dcost_time[index_station][time_minutes][0]
            dtime = dcost_time[index_station][time_minutes][1]
        else : 
            continue

        #Update optimum_path if this station has a lower cost or if the cost is equal but the time is lower.
        if (optimum_path[0] > dcost or (optimum_path[0] == dcost and optimum_path[2] > dtime)) and dcost> 0:

            optimum_path=(dcost, index_station, dtime)
            target_time = time_minutes

    #If no valid interception path is found, return None. 
    if optimum_path == (math.inf, 0, 0):
        return None

    target_station = optimum_path[1]
   
    #return the chosen path and output from the path_backtrack function
    #path_backtrack() time and space complexity : O(d*|L|), where |L| is the number of location available and d = |T|*(the time between stations), where |T| is the number of station,
    return (optimum_path[0], optimum_path[1], optimum_path[-1], path_backtrack(path, target_station, start, target_time))

def path_backtrack(path: list[list[tuple]], target: int, start: int, target_time) -> list:
    """
		Function description: This function is used to reconstructs the path (output of modified_dijkstra function) from the chosen optimum target station (output of optimum_path function)  
                                to the source location (the driver start location), using the path matrix given to this function.
		Approach description: This function reconstructs the path from a target station back to the starting driver location by following the recorded parent pointers stored in the path list. 
                                Each element in path is indexed by [location][time state] and stores a tuple containing (the previous location, and that pevious location time state).
                                Starting from the target and target_time, the function appends each backtracked location to a new list until it reaches the start location (start input) or no valid previous state is found. 
                                The result is then reversed to produce the correct order from the starting point to the destination station (target station).

		Input: 
            path (list[list[tuple]]) : An array based list of list (matrix) containing a tuple of (the previous location, that pevious location time state) in each [location][time state] element index in the matrix. 

            target (int) : is the chosen optimum target station ID (output of the optimum_path function) 

            start (int) : the start location ID of the driver 

            target_time (int) : is the chosen optimum target station time state (output of the optimum_path function) 

		Output: An array based list containing the reconstructed path from the start location to the target station.
		
		Time complexity: O(d*|L|), where |L| is the number of location available and d is the time it takes to loop back to the source station or worst case d= |T|*(the time between stations), where |T| is the number of station in station_time_list.

		Time complexity analysis : Given |L| is the number  number of location available,
                                    Because this function reconstructs the path from a target station back to the starting driver location by following the recorded parent pointers stored in the path list. 
                                    The worst case scenario would be to visit every instance in path, where the number of element/instances in path is O(d*|L|), 
                                    where d is the number of time state and |L| is the number of location available. The value of d = |T|*(the time between stations), where |T| is the number of station.
                                    Therefore, the time complexity is O(d*|L|).
		
                                    Note : time state is the modular time state for each station, meaning that for the driver to intercept their friends at a certain station, assume station A
                                            the driving time between the start location to that station A must : (the driving time between the start location to that station) % (the time needed for the train to loop back) = the time from the friend start station to statioan A

		Space complexity: O(d*|L|),  where |L| is the number of location available and d is the time it takes to loop back to the source station or worst case d= |T|*(the time between stations), where |T| is the number of station in station_time_list.

		Space complexity analysis: Given |L| is the number  number of location available,
                                    Because this function reconstructs the path from a target station back to the starting driver location by following the recorded parent pointers stored in the path list. 
                                    The worst case scenario would be to visit append every instance in path to the new list, where the number of element/instances in path is O(d*|L|), 
                                    where d is the number of time state and |L| is the number of location available. The value of d = |T|*(the time between stations), where |T| is the number of station,
                                    Therefore, the space complexity is O(d*|L|)

                                    Note : time state is the modular time state for each station, meaning that for the driver to intercept their friends at a certain station, assume station A
                                            the driving time between the start location to that station A must : (the driving time between the start location to that station) % (the time needed for the train to loop back) = the time from the friend start station to statioan A

		
	"""
   
    new_path = []
    
    #Iterate until target is None
    #Time: O(d*|L|), Space: O(d*|L|), where |L| is the number of possible location and d = |T|*(the time between stations), where |T| is the number of station.
    while target is not None:
        #Append each location that the pointer pointed
        new_path.append(target)

        #Base case if the pointed instances/element in path is None
        if path[target][target_time] != None:
            #Base case if pointed instances/element in path is not itself (Loop back) or pointed instances/element in path is not the start location tuple
            if path[target][target_time] != (target, target_time) or path[target][target_time] != (start, 0):
                #Recounstruct the path by backtracking the previous pointer/location
                prev, prev_time = path[target][target_time]
                target = prev
                target_time = prev_time
                path[target][target_time] = path[target][target_time]
            else : 
                break
        else : 
            break

    #Reverse the reconstructed path, so that it starts from the start location
    #Time: O(|L|), Auxilary Space: O(1),  Space: O(|L|), where |L| is the number of location
    new_path.reverse()

    return new_path

##############################################################################################################################################################
