import random
import math

class Task:
    def __init__(self, task_id, positions, start_time, end_time):
        self.task_id = task_id
        self.positions = positions
        self.start_time = start_time
        self.end_time = end_time
        self.assigned_drone = None

class Drone:
    def __init__(self, drone_id, position, max_time, velocity):
        self.drone_id = drone_id
        self.position = position
        self.max_time = max_time
        self.bids = {}
        self.assigned_tasks = []
        self.velocity = velocity
        self.ordered_tasks = []
        
    def place_bid(self, task, bid):
        self.bids[task.task_id] = bid

    def calculate_distance(self, task):
        x1, y1 = self.position
        x2, y2 = task.positions[0]
        x3, y3 = task.positions[1]
        distance_a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        distance_b = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
        print(distance_a, distance_b "Task", task.task_id, "Drone", self.drone_id)        
        return min(distance_a, distance_b) 

    def estimate_bid(self, task):
        distance = self.calculate_distance(task)
        if distance == 0:
            bid = float('inf')  # Assign a high bid if the distance is zero
        else:
            bid =  self.velocity / distance  # Lower distance results in a higher bid
        return bid

class Auction:
    def __init__(self, tasks, drones):
        self.tasks = tasks
        self.drones = drones

    def run_auction(self):
        # Bidding phase
        for task in self.tasks:
            for drone in self.drones:
                bid = drone.estimate_bid(task)
                drone.place_bid(task, bid)

        # Assignment phase
        for task in self.tasks:
            max_bid = 0
            max_bidder = None

            for drone in self.drones:
                if task.task_id in drone.bids and drone.bids[task.task_id] > max_bid and task.assigned_drone is None:
                    max_bid = drone.bids[task.task_id]
                    max_bidder = drone

            if max_bidder is not None:
                # Check if the assignment respects the time constraints
                if self.check_time_constraints(max_bidder, task):
                    task.assigned_drone = max_bidder
                    max_bidder.bids.pop(task.task_id)
                else:
                    print(f"Task {task.task_id} cannot be assigned to Drone {max_bidder.drone_id} due to time constraints.")
        

    def check_time_constraints(self, drone, task):
        total_travel_time = 0
        current_position = drone.position

        for position in task.positions:
            distance = self.calc_distance(current_position, position)
            total_travel_time += distance
            current_position = position

        # Check if the total travel time is within the drone's maximum time
        if total_travel_time <= drone.max_time:
            return True
        else:
            return False

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

# Rest of the code remains the same

drone_count = 3
drones = []
drone_x_pos = [1, 2, 3, 3, 8, 3, 4]
drone_y_pos = [0, 0, 0, 0, 0, 0, 0]
for i in range(drone_count):   
    drone = Drone("drone"+str(1+i), (drone_x_pos[i], drone_y_pos[i]), max_time=random.randint(5, 15), 10) 
    drones.append(drone)

tasks_lst = []
for i in range(1, 25):
    xy = []
    xy.append((i, 0))
    xy.append((i, 10))
    tasks_lst.append(xy)

# Create tasks with positions and time constraints
tasks = [Task(task_id, positions, start_time=random.randint(0, 10), end_time=random.randint(20, 30)) for task_id, positions in enumerate(tasks_lst)]

# Create an auction instance and run the auction
auction = Auction(tasks, drones)
auction.run_auction()

# Print the assignments
auction.print_assignments()
