import random
import math

class Task:
    def __init__(self, task_id, positions, start_time, end_time):
        self.task_id = task_id
        self.positions = positions
        self.start_time = start_time
        self.end_time = end_time
        self.assigned_drone = None
        self.start_task_time = 0

class Drone:
    def __init__(self, drone_id, position, max_time, velocity):
        self.drone_id = drone_id
        self.position = position
        self.max_time = max_time
        self.bids = {}
        self.bundle = []
        self.velocity = velocity
        self.path = []
        self.covered_distance = 0
        self.start_pos = ()
        self.end_pos = ()
        self.static_score = 1
        self.update_distance = 0
        self.dicounted_factor = 0.9

    def place_bid(self, task, bid):
        self.bids[task.task_id] = bid

    def calculate_distance(self, task):
        x1, y1 = self.position
        x2, y2 = task.positions[0]
        x3, y3 = task.positions[1]
        distance_a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        distance_b = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
        #print(distance_a, distance_b ,"Task", task.task_id, "Drone", self.drone_id)        
        min_distance = min(distance_a, distance_b)        
        if min_distance == distance_a:
            self.start_pos = task.positions[0]
            self.end_pos = task.positions[1]
        else:
            self.start_pos =  task.positions[1]
            self.end_pos = task.positions[0]
        #print("MIN DISTANCE", min_distance)
        return min_distance

    def estimate_bid(self, task):
        covered_distance = self.covered_distance + self.calculate_distance(task)
        if covered_distance == 0:
            bid = float('inf')  
        else:
            bid =  self.velocity / covered_distance  # Lower distance results in a higher bid
        #print("BID", self.drone_id, task.task_id, bid, "Covered_Distance", covered_distance)
        self.update_distance = covered_distance
        return bid


    def update_bundle(self, task):
        self.bundle.append(task)   

    def update_path(self, task):
        self.path.append(task) 

    def marginal_score_improvement(self, task):
        if task in self.bundle:
            return 0 
        else:
            Spi_i = self.calculate_total_reward(self.path)  
            max_score_improvement = 0
            insert_new_path = []

            for n in range(len(self.path) + 1):
                new_path = self.path[:n] + [task] + self.path[n:]
                new_Spi_i = self.calculate_total_reward(new_path)
                score_improvement = new_Spi_i - Spi_i
                print("Score, n",  score_improvement, n)
                if score_improvement >= max_score_improvement:
                    insert_new_path = new_path
                    max_score_improvement = score_improvement
                    #print("Insert New path",insert_new_path)

            self.bundle = insert_new_path
            self.path = insert_new_path
            for path in insert_new_path:
                pass
                #print("final",path.task_id)
            return max_score_improvement

    def calculate_total_reward(self, path):
        Si = 0
        print("")
        for task in path:
            #print("Task Start Time- Task id", task.start_task_time, task.task_id)
            Si = ((self.dicounted_factor ** task.start_task_time) + Si) * self.static_score
        return Si

class Auction:
    def __init__(self, tasks, drones):
        self.tasks = tasks
        self.drones = drones

    def run_auction(self):
        for task in self.tasks:
            for drone in self.drones:
                bid = drone.estimate_bid(task)
                drone.place_bid(task, bid)

            max_bid = 0
            max_bidder = None

            for drone in self.drones:
                if task.task_id in drone.bids and drone.bids[task.task_id] > max_bid and task.assigned_drone is None:
                    max_bid = drone.bids[task.task_id]
                    max_bidder = drone

            if max_bidder is not None:
                print("Winner", max_bidder.drone_id)
                task.assigned_drone = max_bidder
                max_bidder.position = max_bidder.end_pos
                max_bidder.covered_distance =  max_bidder.update_distance + self.calc_distance(max_bidder.start_pos, max_bidder.end_pos)
                max_bidder.bids.pop(task.task_id) 
                print("Covered Distance", max_bidder.update_distance)
                task.start_task_time = max_bidder.update_distance / max_bidder.velocity
                print("Start Task Time", task.start_task_time)
                marginal_score = max_bidder.marginal_score_improvement(task)
                print(f"Marginal Score Improvement for Drone {max_bidder.drone_id}: {marginal_score}")
                '''if marginal_score >= 1:
                    max_bidder.bundle.append(task)
                    max_bidder.update_path(task)
                    #self.consensus_phase(task, max_bidder)
                '''
        total_distance_travelled = 0
        for drone in drones:
            total_distance_travelled = total_distance_travelled + drone.update_distance
        print("Total Covered_Distance Drones", total_distance_travelled)

    def consensus_phase(self, task, winning_drone):
        updated_tasks = [task] + winning_drone.bundle
        for drone in self.drones:
            if drone != winning_drone:
                self.update_drone_bundle(drone, updated_tasks)

    def update_drone_bundle(self, drone, updated_tasks):
        for task in drone.bundle:
            if task not in updated_tasks:
                drone.bundle.remove(task)
                drone.update_path(task)
                drone.bids[task.task_id] = 0

    def print_assignments(self):
        for task in self.tasks:
            drone_id = task.assigned_drone.drone_id if task.assigned_drone is not None else "Unassigned"
            print(f"Task {task.positions} assigned to Drone {drone_id}")
        self.append_task()

    def append_task(self):
        add_task = []
        for task in self.tasks:
            add_task.append(task.positions)
        print(add_task)
        self.execute_task(add_task)

    def execute_task(self, assigned_task):
        overall_distance = 0
        turns = 0
        if assigned_task is not None:
            print("ditto",assigned_task)
            assigned_task = self.swap_coordinates(assigned_task)
            for i in range(len(assigned_task) - 1):
                distance_travelled = self.calc_distance(assigned_task[i], assigned_task[i+1])
                overall_distance = overall_distance + distance_travelled
                if (assigned_task[i][0] == assigned_task[i+1][0]):
                    turns = turns + 1

            print("Distance Travelled:", overall_distance)
            print("No of Turns", turns)
        return overall_distance, turns

    def calc_distance(self, pos1, pos2):
            x1, y1 = pos1
            x2, y2 = pos2
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return distance

    def swap_coordinates(self, tasks):
        for i in range(len(tasks)):
            if i % 2 != 0:
                tasks[i][0], tasks[i][1] = tasks[i][1], tasks[i][0] 
        combined_tasks = [coord for sublist in tasks for coord in sublist]
        return combined_tasks


drone_count = 2
drones = []
drone_x_pos = [1, 2, 3, 4, 8, 3, 4]
drone_y_pos = [0, 0, 0, 0, 0, 0, 0]
for i in range(drone_count):   
    drone = Drone("drone"+str(1+i), (drone_x_pos[i], drone_y_pos[i]), max_time=random.randint(5, 15), velocity = 10) 
    drones.append(drone)

tasks_lst = []
for i in range(1, 14):
    xy = []
    xy.append((i, 0))
    xy.append((i, 10))
    tasks_lst.append(xy)

tasks = [Task(task_id, positions, start_time=random.randint(0, 10), end_time=random.randint(20, 30)) for task_id, positions in enumerate(tasks_lst)]

auction = Auction(tasks, drones)
auction.run_auction()

auction.print_assignments()
