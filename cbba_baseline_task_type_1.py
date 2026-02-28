#!/usr/bin/python

import matplotlib
matplotlib.use('Agg')
import time
import random
import math
import copy
import matplotlib.pyplot as plt


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
    def __init__(self, drone_id, position, max_time, velocity):
        self.drone_id = drone_id
        self.current_pos = position
        # limits
        self.max_time = max_time
        self.velocity = velocity
        # bundle and path        
        self.bundle = []
        self.path = []
        # bidding params
        self.static_score = 1
        self.update_distance = 0
        self.dicounted_factor = 0.9
        # keep track of best bid task, path, bundle
        self.bids = {}
        self.insert_new_path = []
        self.insert_new_bundle = []

    def place_bid(self, task, bid):
        self.bids[task.task_id] = bid

    def calculate_distance(self, path, i, curr_pos):
        end_pos = curr_pos
        
        x1, y1 = curr_pos
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
            #distance += task.distance
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
            #distance += task.distance
            #print(self.drone_id, "Task", task.positions, "Distance =",  distance, "current_pos =", curr_pos)
        return distance

class Auction:
    def __init__(self, tasks, drones):
        self.tasks = copy.copy(tasks)
        self.drones = drones
        self.tasks_backup = copy.copy(tasks)
        self.start_time = time.time()
        self.end_time = 0


    def estimate_bid(self, drone):
        # max bid task and the max bid
        bid, task = drone.marginal_score_improvement(self.tasks)           
        if task is not None:  
            #print("Drone", drone.drone_id, "Bid:", bid, "Task:", task.task_id)
            drone.place_bid(task, bid)

    def run_auction(self):
        unassigned_tasks = copy.copy(self.tasks)

        while True:
            if len(unassigned_tasks) == 0:
                break

            # Let each drone find the best task to bid on and place a bid
            for drone in drones:
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
                #print ("drone: ", drone.drone_id)
                #print ("bids: ", drone.bids)
                #print("Distance Covered", drone.distance)

            for task in actively_bid_tasks:
                # find best bid
                highest_bid = -1
                winning_drone = None

                for drone in drones:
                    if task.task_id in drone.bids:
                        if drone.bids[task.task_id] > highest_bid:
                            highest_bid = drone.bids[task.task_id]
                            winning_drone = drone
                
                if winning_drone is not None:
                    #print ("task: ", task.task_id, "drone: ", winning_drone.drone_id)
                    task.assigned_drone = winning_drone
                    winning_drone.assign_task()
                    # remove task from unassigned
                    unassigned_tasks.remove(task)
            
            # remove existing bids of all drones
            for drone in drones:
                drone.reset_bids() 

            #print ("-----------------------\n")
        
        self.end_time = time.time()
        print("Execution Time:", self.end_time - self.start_time, "seconds")         
            
    def print_assignments(self):
        for task in self.tasks:
            drone_id = task.assigned_drone.drone_id if task.assigned_drone is not None else "Unassigned"
            print(f"Task {task.positions} assigned to Drone {drone_id}")

    def calc_distance(self, pos1, pos2):
            x1, y1 = pos1
            x2, y2 = pos2
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return distance

    def metrics(self, drones):
        overall_distance = 0
        for drone in drones:
            distance_covered = drone.calculate_covered_distance(drone.path) * 0.5
            print(drone.drone_id, "Distance Covered =", distance_covered)
            overall_distance = overall_distance + distance_covered
        print("Over all distance covered by drone", overall_distance)

    def calculate_fairness_score(self, drones, tasks):
        total_tasks = len(tasks)
        fair_tasks_per_drone = total_tasks / (len(drones))

        fairness_scores = []

        for drone in drones:
            assigned_tasks = [task for task in tasks if task.assigned_drone == drone]
            num_assigned_tasks = len(assigned_tasks)
            fairness_score = abs(num_assigned_tasks - fair_tasks_per_drone)
            print("Fairness Score", fairness_score, drone.drone_id)
            fairness_scores.append((drone, fairness_score))

        print("Fairness Score", fairness_scores)
        return fairness_scores
    
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
        plt.savefig('tasks_cbba_base.png')

if __name__ == "__main__":
    drone_count = 5
    drones = []
    drone_x_pos = [5, 1, 8, 2, 1, 3, 4]
    drone_y_pos = [8, 0, 0, 9, 7, 10, 0]
    velocity = [10, 10, 10, 10, 10]
    for i in range(drone_count):   
        drone = Drone("drone"+str(1+i), (drone_x_pos[i], drone_y_pos[i]), max_time=random.randint(5, 15), velocity = velocity[i]) 
        drones.append(drone)

    '''for i in range(1, 31):
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
    tasks = [Task(task_id, positions) for task_id, positions in enumerate(tasks_lst)]

    auction = Auction(tasks, drones)
    auction.run_auction()
    auction.print_assignments()
    auction.metrics(drones)
    auction.visualize()
    auction.calculate_fairness_score(drones, tasks)