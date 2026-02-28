import airsim
import cv2
import numpy as np
import os
import pprint
import setup_path 
import tempfile
import time

# Use below in settings.json with Blocks environment
# ... (settings.json configuration remains the same)

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")



airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name="Drone1")
f2 = client.takeoffAsync(vehicle_name="Drone2")
f1.join()
f2.join()


state1 = client.getMultirotorState(vehicle_name="Drone1")
s = pprint.pformat(state1)
print("state: %s" % s)
state2 = client.getMultirotorState(vehicle_name="Drone2")
s = pprint.pformat(state2)
print("state: %s" % s)

# ... (code for takeoff and initial state)

airsim.wait_key('Press any key to move vehicles')

# Move drones to specified positions
positions = [
    (0, 1, -10),
    (1, 1, -10),
    (0, 10, -10),
    (1, 10, -10),
    (0, 20, -10),
    (1, 20, -10),
    (0, 30, -10),
    (1, 30, -10),
    (0, 40, -10),
    (1, 40, -10)
]
path = []
count = 0
for position in positions:
    path.append(airsim.Vector3r(position[0], position[1], position[2]))
    f1 = client.moveToPositionAsync(position[0], position[1], position[2], 5, vehicle_name="Drone1")
    f2 = client.moveToPositionAsync(position[0] + 1, position[1], position[2], 5, vehicle_name="Drone2")
    f1.join()
    if count <= 5:
        print("Inside")
        f2.join()
    count = count + 1
    print("Moved to position:", position)

result = client.moveOnPathAsync(path, 5, 50, airsim.DrivetrainType.ForwardOnly, 
                airsim.YawMode(False,0), 5 + (5/2), 1).join()
airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")