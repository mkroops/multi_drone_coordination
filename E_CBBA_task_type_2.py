#!/usr/bin/python

import matplotlib
matplotlib.use('Agg')
import time
import random
import math
import copy
import matplotlib.pyplot as plt


class Task:
    def __init__(self, task_id, positions, start_time, end_time):
        self.task_id = task_id
        self.positions = positions
        self.start_pos = ()
        self.end_pos = ()
        self.assigned_drone = None
        self.start_task_time = 0
        self.updated_task_pos = ()
        self.distance = 0.0
        self.calc_distance()
    
    def calc_distance(self):
        self.distance = math.sqrt((self.positions[1][0] - self.positions[0][0])**2 +
                                  (self.positions[1][1] - self.positions[0][1])**2)

class Drone:
    def __init__(self, drone_id, is_am_queen, position, max_time, velocity, energy):
        self.is_am_queen = is_am_queen
        self.start_pos_bool = False
        self.drone_id = drone_id
        self.current_pos = position
        self.energy = energy
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
        pos = curr_pos
        
        x1, y1 = curr_pos
        task = path[i]
        x2, y2 = task.positions[0]
        x3, y3 = task.positions[1]
        distance_a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        distance_b = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)


        min_distance = min(distance_a, distance_b)

        if self.start_pos_bool == True:
            pos = task.positions[0] if min_distance == distance_a else task.positions[1]
        else:
            pos = task.positions[1] if min_distance == distance_a else task.positions[0]

        return (min_distance, pos)

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
        update_pos = copy.copy(self.current_pos)
        minimum_dist = []
        for j in range(len(assigned_tasks)):
            _ , curr_pos = self.calculate_distance(assigned_tasks, j, update_pos)
            dist_a = 0 
            dist_b = 0
            diff = 0
            for i in range(len(assigned_tasks)):
                dist, _ = self.calculate_distance(assigned_tasks, i, curr_pos)
                if assigned_tasks[i].task_id < assigned_tasks[j].task_id:
                    dist_a = dist_a + dist
                else:
                    dist_b = dist_b + dist
                
                
            diff = abs(dist_a - dist_b) 
            minimum_dist.append(diff)
            #print("Queen", assigned_tasks[j].task_id, "diff", diff)
            update_pos = curr_pos

        self.start_pos_bool = False
        index = minimum_dist.index(min(minimum_dist))
        #print("Index", index, assigned_tasks[index].task_id)
        assigned_tasks[index].assigned_drone = self

        delta_distance_to_start, queen_curr_pos = self.calculate_distance(assigned_tasks, index, self.current_pos)
        self.distance += delta_distance_to_start
        self.distance += assigned_tasks[index].distance
        self.current_pos = queen_curr_pos
        #self.energy = self.energy - (self.distance * 0.01)
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

        # path before insertion is same for all tasks. find this reward only once
        Spi_i = self.calculate_total_reward(self.path)  

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
                    new_Spi_i = self.calculate_total_reward(new_path)
                    score_improvement = new_Spi_i - Spi_i
                    #print("Score =", score_improvement, "Task=", task.task_id  ,"Combinations =",n)
                    if score_improvement >= best_bid:
                        best_task = task
                        best_bid = score_improvement
                        insert_new_path = new_path
                        inserted_position = copy.copy(n)
                        best_new_Spi_i = new_Spi_i
        
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
            '''print ("drone", self.drone_id,
                   "No best task to bid on")'''
        #self.energy = self.energy - (self.distance * 0.01)
        return best_bid, best_task # best task could be None

    def calculate_total_reward(self, path):
        Si = 0
        start_time = 0.0
        curr_pos = copy.copy(self.current_pos)
        distance = 0.0
        delta_distance_to_start = 0.0

        for i in range(len(path)):
            task = path[i]
            delta_distance_to_start, curr_pos  = self.calculate_distance(path, i, curr_pos)
            distance += delta_distance_to_start
            start_time = distance / self.velocity
            Si += ((self.dicounted_factor ** start_time)) * self.static_score
            # add task distance
            distance += task.distance
            self.distance = distance

        '''print ("")
        print ("drone: ", self.drone_id)
        print ("path: ", [task.task_id for task in path])
        print ("distance (including distance of all tasks in path): ", distance)
        print ("Si: ", Si)
        print ("")'''
        return Si
    
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

    def calculate_energy_efficiency(self, drone, task):
        energy_consumption = self.calc_distance(drone.current_pos, task.positions) + task.distance
        energy_efficiency = 1 / energy_consumption if energy_consumption != 0 else 0
        return energy_efficiency

    def calc_distance(self, pos1, pos2):
            if len(pos2) == 2:
                x1, y1 = pos1
                positions = pos2
                x2, y2 = positions[0]
                x3, y3 = positions[1]
                distance_a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                distance_b = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
                distance = min(distance_a, distance_b)
            else:
                x1, y1 = pos1
                x2, y2 = pos2
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return distance

class Auction:
    def __init__(self, tasks, drones):
        self.tasks = copy.copy(tasks)
        self.drones = copy.copy(drones)
        self.assigned_tasks = []
        self.tasks_backup = copy.copy(tasks)
        self.start_time = time.time()
        self.end_time = 0

    def estimate_bid(self, drone):
        if drone.is_am_queen == 'Queen':
            self.start_pos_bool = True
            bid , task = drone.estimate_best_task_for_queen(self.assigned_tasks)
            drone.energy = drone.energy - self.calculate_energy_efficiency(drone, task)
            #print("Task ID", task.task_id )
            task.assigned_drone = drone
            drone.queen_bundle.append(task)
            return
        else:
            bid, task = drone.marginal_score_improvement(self.tasks)           
        if task is not None: 
            composite_reward = self.calculate_composite_reward(drone, task)
            #print("Drone", drone.drone_id, "Bid:", composite_reward, "Task:", task.task_id) 
            #print("Drone", drone.drone_id, "Bid:", bid, "Task:", task.task_id)
            normalized_drone_energy = (drone.energy - 0) / (100 - 0) 
            bid =  composite_reward
            #print("BID", bid, normalized_drone_energy)
            drone.place_bid(task, bid)
        else:
            return

    def calculate_energy_efficiency(self, drone, task):
        energy_consumption = self.calc_distance(drone.current_pos, task.positions) + task.distance
        energy_efficiency = 1 / energy_consumption if energy_consumption != 0 else 0
        return energy_efficiency

    def calculate_completion_time(self, drone, task):
        time_to_reach_task = self.calc_distance(drone.current_pos, task.positions) / drone.velocity
        completion_time = time_to_reach_task + task.distance / drone.velocity
        return completion_time

    def calculate_fairness(self, drone, task):
        tasks_assigned_to_drone = len([t for t in self.assigned_tasks if t.assigned_drone == drone])
        fairness = 1 / (tasks_assigned_to_drone + 1)  
        return fairness

    def calculate_composite_reward(self, drone, task):
        energy_efficiency = self.calculate_energy_efficiency(drone, task)
        energy_efficiency = (drone.energy - energy_efficiency)
        energy_efficiency = (energy_efficiency - 0) / (100 - 0)
        completion_time = (self.calculate_completion_time(drone, task))
        completion_time = 1
        fairness = self.calculate_fairness(drone, task)
        #print("Completion Time", completion_time, "Fairness", fairness)
        #composite_reward =  energy_efficiency * completion_time * fairness

        weight_energy = 0.9
        weight_time = 0.2
        weight_fairness = 0.6
        
        '''composite_reward = (
            weight_energy * energy_efficiency
            + weight_time * (1 / completion_time)
            + weight_fairness * fairness
        )'''

        composite_reward = (
            weight_energy * energy_efficiency
            + weight_fairness * fairness
        )

        return composite_reward


    def run_auction(self):
        unassigned_tasks = copy.copy(self.tasks)
        task_threshold = int(0.3 * len(self.tasks))
        while True:
            #print("Threshold", task_threshold)
            for drone in self.drones:
                if drone.energy < 10:
                    self.drones.remove(drone) 
            
            if len(self.drones) <= 1:
                break
                
            if len(unassigned_tasks) == 0:
                break

            for drone in self.drones:
                if drone.is_am_queen == 'Queen':
                    #print(len(self.assigned_tasks))
                    if (len(self.assigned_tasks)) >= task_threshold:
                        self.estimate_bid(drone)
                        self.assigned_tasks = []
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
                '''print("")
                print ("drone: ", drone.drone_id)
                print ("bids: ", drone.bids)
                print("Distance Covered", drone.distance)
                print("Drone Energy", drone.energy)'''

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
                    winning_drone.energy = winning_drone.energy - self.calculate_energy_efficiency(winning_drone, task)
                    winning_drone.assign_task()
                    # remove task from unassigned
                    self.assigned_tasks.append(task)
                    unassigned_tasks.remove(task)
            
            # remove existing bids of all drones
            for drone in drones:
                drone.reset_bids() 

            #print ("-----------------------\n")        

        self.end_time = time.time()
        print("Execution Time:", self.end_time - self.start_time, "seconds") 
        


    def print_assignments(self):
        plt.figure(figsize=(10, 6))
        for task in self.tasks:
            drone_id = task.assigned_drone.drone_id if task.assigned_drone is not None else "Unassigned"
            print(f"Task {task.positions} assigned to Drone {drone_id}")

    def calc_distance(self, pos1, pos2):
            if len(pos2) == 2:
                x1, y1 = pos1
                positions = pos2
                x2, y2 = positions[0]
                x3, y3 = positions[1]
                distance_a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                distance_b = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
                distance = min(distance_a, distance_b)
            else:
                x1, y1 = pos1
                x2, y2 = pos2
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return distance

    def metrics(self, drones):
        overall_distance = 0
        for drone in drones:
            if drone.is_am_queen == 'Queen':
                distance_covered = drone.distance
            else:
                distance_covered = drone.calculate_covered_distance(drone.path)
            print(drone.drone_id, "Distance Covered =", distance_covered)
            overall_distance = overall_distance + distance_covered
        print("Over all distance covered by drone", overall_distance)


    def visualize(self):
        drone_colors = ['c', '#000000', 'g', 'r', 'b', 'm', 'y', 'k']

        plt.figure(figsize=(12, 6))

        for task in tasks:
            if task.assigned_drone is not None:
                drone = task.assigned_drone
                x_vals = [pos[0] for pos in task.positions]
                y_vals = [pos[1] for pos in task.positions]
                i = int(''.join(filter(str.isdigit, drone.drone_id)))
                plt.plot(x_vals, y_vals, marker='o', label=f'Task {task.task_id} - Drone {drone.drone_id}', color=drone_colors[i])

        '''for i, drone in enumerate(drones):
            x_vals = [pos[0] for pos in drone.path_coordinates]  # Use path_coordinates
            y_vals = [pos[1] for pos in drone.path_coordinates]  # Use path_coordinates
            k = int(''.join(filter(str.isdigit, drone.drone_id)))
            plt.plot(x_vals, y_vals, linestyle='dashed', label=f'Drone {drone.drone_id} Path', color=drone_colors[k])'''

        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Task Assignments and Drone Paths')
        plt.legend()
        plt.savefig('test_mod_cbba_baseline.png')


if __name__ == "__main__":
    drone_count = 5
    drones = []
    drone_x_pos = [1, 2, 3, 4, 5, 3, 4]
    drone_y_pos = [0, 0, 0, 0, 0, 0, 0]
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
    k = 20
    j = 21
    for i in range(1, 31):
        '''xy = []
        if i <= 20:
            k = k - 1
            j = j + 1
            xy.append((i, k))
            xy.append((i, j))
        else:
            k = k + 1
            j = j - 1
            xy.append((i, k))
            xy.append((i, j))
        tasks_lst.append(xy)'''
        xy = []
        xy.append((i, 0))
        xy.append((i, 10))
        tasks_lst.append(xy)


    tasks = [Task(task_id, positions, start_time=random.randint(0, 10), end_time=random.randint(20, 30)) for task_id, positions in enumerate(tasks_lst)]

    auction = Auction(tasks, drones)
    auction.run_auction()

    auction.print_assignments()
    auction.metrics(drones)
    auction.visualize()