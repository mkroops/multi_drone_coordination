#!/usr/bin/python

import matplotlib
matplotlib.use('Agg')
from sklearn.cluster import KMeans
import numpy as np
import random
import math
import copy
import matplotlib.pyplot as plt
import time


class Task:
    def __init__(self, task_id, positions):
        self.task_id = task_id
        self.positions = positions
        self.start_pos = ()
        self.end_pos = ()
        self.assigned_drone = None
        self.start_task_time = 0
        self.updated_task_pos = ()
        self.distance = 0.0


class Drone:
    def __init__(self, drone_id, is_am_queen, position, max_time, velocity, energy):
        self.is_am_queen = is_am_queen
        self.start_pos_bool = False
        self.drone_id = drone_id
        self.current_pos = position
        self.energy = 200 * 3600 * 1000 * (energy/100)
        # limits
        self.max_time = max_time
        self.velocity = velocity
        # bundle and path        
        self.bundle = []
        self.path = []
        # bidding params
        self.static_score = 1
        self.distance = 0
        self.dicounted_factor = 0.9
        # keep track of best bid task, path, bundle
        self.bids = {}
        self.insert_new_path = []
        self.insert_new_bundle = []
        self.queen_bundle = []

        self.path_coordinates = []


    def place_bid(self, task, bid):
        self.bids[task.task_id] = bid

    def calculate_distance(self, path, i, curr_pos):
        end_pos = curr_pos
        
        x1, y1 = curr_pos
        if i is not None: 
            task = path[i]
        x2, y2 = task.positions
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        end_pos = task.positions

        return (distance, end_pos)

    def assign_task(self):
        self.bundle = self.insert_new_bundle
        self.path = self.insert_new_path
    
    def reset_bids(self):
        self.bids = {}
        self.insert_new_bundle = []
        self.insert_new_path = []

    def update_path(self, task):
        self.path.append(task) 
        self.path_coordinates.append(self.current_pos)

    def estimate_best_task_for_queen(self, assigned_tasks):
        curr_pos = copy.copy(self.current_pos)
        minimum_dist = []

        tasks = []
        for task in assigned_tasks:
            tasks.append(task.positions)
        data = np.array(tasks)
        num_clusters = 1

        kmeans = KMeans(n_clusters=num_clusters)
        kmeans.fit(data)
        cluster_centers = kmeans.cluster_centers_
        cluster_centers = [tuple(center) for center in cluster_centers]
        cluster_center = cluster_centers[0]
        cluster_center =  tuple(int(round(val)) for val in cluster_center)

        index = None
        for i, task in enumerate(assigned_tasks):
            if task.positions == cluster_center:
                index = i
                break
        
        distance, queen_curr_pos = self.calculate_distance(assigned_tasks, index, self.current_pos)
        self.distance += distance
        self.current_pos = queen_curr_pos
        return 0, assigned_tasks[index]

    def marginal_score_improvement(self, tasks):
        """ Current strategy: more like cbba
        This will find the best task to bid on for the drone and places a bid.
        It is possible that only one drone will be bidding on this task. 
        
        Alternate strategy: proper single item auction 
        Calculate the best bid for a single task by the drone in this function and 
        compare it across the drone team to find the best possible bid for that task 
        at the latest stage of allocations.
        """
        best_task = None
        best_bid = 0
        insert_new_path = None
        inserted_position = None
        best_new_Spi_i = 0
        energy_for_path = 0

        # path before insertion is same for all tasks. find this reward only once
        Spi_i, energy_consumption = self.calculate_total_reward(self.path)  

        for task in tasks:
            if task in self.bundle:
                # return 0 # this method should return two values. returning zero is wrong. will cause the drone to miss some tasks
                continue 
            elif task.assigned_drone is not None:
                # already assigned to another drone
                continue
            else:
                # Spi_i = self.calculate_total_reward(self.path)  # There is no need to calculate this again and again

                for n in range(len(self.path) + 1):
                    new_path = self.path[:n] + [task] + self.path[n:]
                    new_Spi_i, energy_consumption = self.calculate_total_reward(new_path)
                    score_improvement = new_Spi_i - Spi_i
                    #print("Score =", score_improvement, "Task=", task.task_id  ,"Combinations =",n)
                    if score_improvement >= best_bid:
                        best_task = task
                        best_bid = score_improvement
                        insert_new_path = new_path
                        inserted_position = copy.copy(n)
                        best_new_Spi_i = new_Spi_i
                        energy_for_path = energy_consumption
        
        if best_task is not None:
            self.insert_new_path = insert_new_path
            self.insert_new_bundle = drone.bundle + [best_task]

            '''print ("drone: ", self.drone_id, 
                "new task: ", best_task.task_id, 
                "inserted at: ", inserted_position, 
                "Spi_i: ", Spi_i, 
                "new Spi_i: ", best_new_Spi_i, 
                "bid: ", best_bid,
                "new path: ", [item.task_id for item in insert_new_path])'''
        else:
            pass
            #print ("drone", self.drone_id,
                   #"No best task to bid on")
        #self.energy = self.energy - (self.distance * 0.01)
        return best_bid, best_task, energy_for_path# best task could be None

    def calculate_total_reward(self, path):
        Si = 0
        start_time = 0.0
        curr_pos = copy.copy(self.current_pos)
        distance = 0.0
        delta_distance_to_start = 0.0
        energy_consumption = 0.0


        for i in range(len(path)):
            task = path[i]
            delta_distance_to_start, curr_pos  = self.calculate_distance(path, i, curr_pos)
            distance += delta_distance_to_start
            start_time = distance / self.velocity
            Si += ((self.dicounted_factor ** start_time)) * self.static_score
            energy_consumption = energy_consumption + self.calculate_energy_efficiency(distance)
            # add task distance
            distance += task.distance
            self.distance = distance

        '''print ("")
        print ("drone: ", self.drone_id)
        print ("path: ", [task.task_id for task in path])
        print ("distance (including distance of all tasks in path): ", distance)
        print ("Si: ", Si)
        print ("")'''
        return Si, energy_consumption
    
    def calculate_covered_distance(self, path):
        
        curr_pos = copy.copy(self.current_pos)
        distance = 0.0
        delta_distance_to_start = 0.0
        
        for i in range(len(path)):
            task = path[i]
            delta_distance_to_start, curr_pos  = self.calculate_distance(path, i, curr_pos)
            distance += delta_distance_to_start
            start_time = distance / self.velocity
            distance += task.distance
            #print(self.drone_id, "Task", task.positions, "Distance =",  distance, "current_pos =", curr_pos)
        return distance

    def calculate_energy_efficiency(self, distance):
        moving_watt = 15
        energy_efficiency = ((moving_watt * (drone.distance/ drone.velocity)) * 1000) 
        #energy_efficiency = 1 / distance if distance != 0 else 0
        return energy_efficiency

    def calc_distance(self, pos1, pos2):
        x1, y1 = pos1
        positions = pos2
        x2, y2 = positions
        distance_a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance

class Auction:
    def __init__(self, tasks, drones):
        self.tasks = copy.copy(tasks)
        self.start_time = time.time()
        self.end_time = 0
        self.drones = copy.copy(drones)
        self.threshold_tasks = []
        self.assigned_tasks = []
        self.fairness_tasks = []
        self.distance_covered = []
        self.tasks_backup = copy.copy(tasks)

    def estimate_bid(self, drone):
        if drone.is_am_queen == 'Queen':
            self.start_pos_bool = True
            bid , task = drone.estimate_best_task_for_queen(self.threshold_tasks)
            drone.energy = drone.energy - self.calculate_energy_efficiency(drone, task)
            #print("Task ID", task.task_id )
            task.assigned_drone = drone
            drone.queen_bundle.append(task)
            return
        else:
            bid, task, energy_for_path = drone.marginal_score_improvement(self.tasks)       
        if task is not None: 
            composite_reward = self.calculate_composite_reward(drone, task, energy_for_path)
            #print("Drone", drone.drone_id, "Bid:", composite_reward, "Task:", task.task_id) 
            #print("Drone", drone.drone_id, "Bid:", bid, "Task:", task.task_id)
            normalized_drone_energy = (drone.energy - 0) / (100 - 0) 
            bid =  composite_reward
            #print("BID", bid, normalized_drone_energy)
            drone.place_bid(task, bid)
            
        else:
            return

    def calculate_energy_efficiency(self, drone, task):
        energy_consumption = self.calc_distance(drone.current_pos, task.positions) 
        energy_efficiency = 1 / energy_consumption if energy_consumption != 0 else 0
        return energy_efficiency

    def calculate_completion_time(self, drone, task):
        time_to_reach_task = self.calc_distance(drone.current_pos, task.positions) / drone.velocity
        completion_time = time_to_reach_task + task.distance / drone.velocity
        return completion_time

    def calculate_fairness(self, drone, task):
            tasks_assigned_to_drone = len([t for t in self.fairness_tasks if t.assigned_drone == drone])
            fairness = 1 / (tasks_assigned_to_drone + 1)  
            return fairness
        
    def calculate_composite_reward(self, drone, task, energy_for_path):
        #energy_efficiency = self.calculate_energy_efficiency(drone, task)
        #print("Energy for path", energy_for_path, "Drone Energy", drone.energy)
        energy_efficiency_path = (drone.energy - (energy_for_path))
        energy_efficiency = (energy_efficiency_path - 0) / ((200 * 3600 * 1000)- 0)
        print("Energy Efficiency", energy_efficiency)
        completion_time = (self.calculate_completion_time(drone, task))
        completion_time = 1
        fairness = self.calculate_fairness(drone, task)
        print("Completion Time", completion_time, "Fairness", fairness)
        #composite_reward =  energy_efficiency * completion_time * self.drones.remove(drone) 

        weight_energy = 0.1
        weight_time = 0.2
        weight_fairness = 0.0
        
        '''composite_reward = (
            weight_energy * energy_efficiency
            + weight_time * (1 / completion_time)
            + weight_fairness * fairness
        )'''

        composite_reward = (
            weight_energy * energy_efficiency
            + weight_fairness * fairness
        )

        if energy_efficiency_path <  (200 * 3600 * 1000 * 0.10):
            self.drones.remove(drone) 
            return 0
        return composite_reward


    def run_auction(self):
        unassigned_tasks = copy.copy(self.tasks)
        task_threshold = int(0.5 * len(self.tasks))
        while True:
            #print("Threshold", task_threshold)
            for drone in self.drones:
                if drone.energy < (200 * 3600 * 1000 * 0.10):
                    self.drones.remove(drone) 
            
            if len(self.drones) <= 1:
                break
                
            if len(unassigned_tasks) == 0:
                break

            for drone in self.drones:
                if drone.is_am_queen == 'Queen':
                    pass
                else:
                    self.estimate_bid(drone)

            actively_bid_tasks = []
            for task in unassigned_tasks:
                actively_bid = sum([1 if task.task_id in drone.bids else 0 for drone in drones]) > 0
                if actively_bid:
                    actively_bid_tasks.append(task)
            
            #print ("")
            #print ("allocating tasks: ", [task.task_id for task in actively_bid_tasks])
            for drone in drones:
                pass
                #print("")
                #print ("drone: ", drone.drone_id, task.task_id)
                #print ("bids: ", drone.bids)
                id = []
                for j in drone.path:
                    id.append(j.task_id)
                #print("Path", id)
                #print("Distance Covered", drone.distance)
                #print("Drone Energy", drone.energy)

            for task in actively_bid_tasks:
                # find best bid
                highest_bid = -1
                winning_drone = None

                for drone in drones:
                    if drone.is_am_queen == 'Queen':
                        continue
                    if task.task_id in drone.bids:
                        if drone.bids[task.task_id] > highest_bid:
                            highest_bid = drone.bids[task.task_id]
                            winning_drone = drone
                
                
                if winning_drone is not None:
                    #print ("task: ", task.task_id, "WINNING DRONE: ", winning_drone.drone_id)
                    task.assigned_drone = winning_drone
                    winning_drone.assign_task()
                    self.fairness_tasks.append(task)
                    # remove task from unassigned
                    unassigned_tasks.remove(task)
            
            # remove existing bids of all drones
            for drone in drones:
                drone.reset_bids() 

            #print ("-----------------------\n")  

        max = 0
        leng = 0
        for drone in self.drones:
            if drone.is_am_queen == 'Queen':
                pass
            else:
                leng = len(drone.path)
            if leng > max:
                max = leng

        assigned_tasks = []
        count = 0
        queen_drone = None
        for i in range(max):
            for drone in self.drones:
                if drone.is_am_queen == 'Queen':
                    queen_drone = drone
                    pass
                else:
                    if i >= len(drone.path):
                        continue
                    else:
                        count = count + 1
                        assigned_tasks.append(drone.path[i])
                        self.assigned_tasks.append(drone.path[i])

                if count == task_threshold:
                    #print("Inside")
                    self.threshold_tasks = assigned_tasks
                    self.estimate_bid(queen_drone)
                    count = 0
                    assigned_tasks = []
        
        self.end_time = time.time()
        print("Execution Time:", self.end_time - self.start_time, "seconds")


    def calculate_fairness_score(self, drones, tasks):
        total_tasks = len(tasks)
        fair_tasks_per_drone = total_tasks / (len(drones))

        fairness_scores = []

        for drone in drones:
            '''if drone.is_am_queen == 'Queen':
                continue'''
            assigned_tasks = [task for task in tasks if task.assigned_drone == drone]
            num_assigned_tasks = len(assigned_tasks)
            fairness_score = abs(num_assigned_tasks - fair_tasks_per_drone)
            print("Fairness Score", fairness_score, drone.drone_id)
            fairness_scores.append((drone, fairness_score))

        print("Fairness Score", fairness_scores)
        return fairness_scores

        
    def calculate_drone_energy_consumption(self, drones):
        max_distance = max(self.distance_covered)
        hovering_watt = 20
        moving_watt = 60
        initial_battery_capacity = 200 * 3600 * 1000  
        for drone in drones:
            hovering_energy = (hovering_watt) * ( (max_distance/10) - (drone.distance/ drone.velocity))
            moving_energy = (moving_watt * (drone.distance/ drone.velocity))
            Energy_consumed =  (moving_energy + hovering_energy) * 1000  
            print("Energy Consumed", Energy_consumed)
            remaining_battery_life = initial_battery_capacity - Energy_consumed
            print("Remaining Battery Life", remaining_battery_life)
            energy_consumed_percentage = (Energy_consumed / initial_battery_capacity) * 100
            print("Battery Percentage", energy_consumed_percentage)

            
    def print_assignments(self):
        plt.figure(figsize=(10, 6))
        for task in self.tasks:
            drone_id = task.assigned_drone.drone_id if task.assigned_drone is not None else "Unassigned"
            print(f"Task {task.positions} assigned to Drone {drone_id}")

    def calc_distance(self, pos1, pos2):
        x1, y1 = pos1
        positions = pos2
        x2, y2 = positions
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance

    def metrics(self, drones):
        overall_distance = 0
        for drone in drones:
            if drone.is_am_queen == 'Queen':
                distance_covered = drone.distance 
                self.distance_covered.append(distance_covered)
            else:
                distance_covered = drone.calculate_covered_distance(drone.path) 
                self.distance_covered.append(distance_covered)
                drone.distance = distance_covered 
            print(drone.drone_id, "No of winning tasks =", len(drone.path))
            print(drone.drone_id, "Distance Covered =", distance_covered)
            overall_distance = overall_distance + distance_covered
        print("Over all distance covered by drone", overall_distance)

    def visualize(self):
        drone_colors = ['c', '#000000', 'g', 'r', 'b', 'm', 'y', 'k']

        plt.figure(figsize=(12, 6))

        for task in tasks:
            if task.assigned_drone is not None:
                drone = task.assigned_drone
                x = task.positions[0]
                y = task.positions[1]
                i = int(''.join(filter(str.isdigit, drone.drone_id)))
                plt.scatter(x, y, color=drone_colors[i], label=drone.drone_id)
                
        for i, drone in enumerate(drones):
            x_vals = task.positions
            k = int(''.join(filter(str.isdigit, drone.drone_id)))
            #plt.plot(x_vals, linestyle='dashed', label=f'Drone {drone.drone_id} Path', color=drone_colors[k])

        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Task Assignments and Drone Paths')
        plt.legend()
        plt.savefig('version_new.png')


if __name__ == "__main__":
    drone_count = 5
    drones = []
    drone_x_pos = [1, 2, 3, 4, 5, 3, 4]
    drone_y_pos = [0, 0, 0, 0, 0, 10, 0]
    velocity = [10, 10, 10, 10, 10]
    drone_energy = [80, 70, 70, 70, 70]
    for i in range(drone_count):
        if( i==0 ):
            drone = Drone("drone"+str(1+i), 'Queen', (drone_x_pos[i], drone_y_pos[i]), max_time=random.randint(5, 15), velocity = velocity[i], energy = drone_energy[i]) 
            drones.append(drone)    
            continue   
        drone = Drone("drone"+str(1+i), 'Bee', (drone_x_pos[i], drone_y_pos[i]), max_time=random.randint(5, 15),velocity = velocity[i], energy = drone_energy[i]) 
        drones.append(drone)

    tasks_lst = []
    '''for i in range(1, 11):
        for j in range(1, 11):
            tasks_lst.append((i, j))'''

    

    tasks_lst = [
        (17,0), (17,1), (17,2), (17,3),
        (16,0), (16,1), (16,2), (16,3),
        (15,0), (15,1), (15,2), (15,3),
        (14,0), (14,1), (14,2), (14,3), (14, 4),
        (13,0), (13,1), (13,2), (13,3), (13, 4),
        (12,0), (12,1), (12,2), (12,3), (12, 4), (12,5),
        (11,0), (11,1), (11,2), (11,3), (11, 4), (11,5), (11, 6), (11,7),
        (10,0), (10,1), (10,2), (10,3), (10, 4), (10,5), (10, 6), (10,7),
        (9,0), (9,1), (9,2), (9,3), (9, 4), (9,5), (9, 6), (9,7),
        (8,0), (8,1), (8,2), (8,3), (8, 4), (8,5), (8, 6), (8,7), (8,8),
        (7,0), (7,1), (7,2), (7,3), (7, 4), (7,5), (7, 6), (7,7), (7,8), (7,9),
        (6,4), (6,5), (6,6), (6,7), (6, 8), (6,9),
        (5,4), (5,5), (5,6), (5,7), (5, 8), (5,9),
        (4,4), (4,5), (4,6), (4,7), (4, 8), (4,9),
        (3,4), (3,5), (3,6), (3,7), (3, 8), (3,9),
        (2,4), (2,5), (2,6), (2,7), (2, 8), (2,9),
        (1,4), (1,5), (1,6), (1,7), (1, 8), (1,9),
        
    ]
    tasks = [Task(task_id, positions) for task_id, positions in enumerate(tasks_lst)]

    auction = Auction(tasks, drones)
    auction.run_auction()
    auction.print_assignments()
    auction.metrics(drones)
    auction.visualize()
    auction.calculate_drone_energy_consumption(drones)
    auction.calculate_fairness_score(drones, tasks)