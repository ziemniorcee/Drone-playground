import pybullet as p
import pybullet_data
import time
import os
import math
# 1. Setup Physics with "Maximi1400zed" Window
# We set a large width/height to fill the screen
physicsClient = p.connect(p.GUI, options='--width=1420 --height=800')
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")

# Load Drone
script_dir = os.path.dirname(os.path.abspath(__file__))
drone_urdf_path = os.path.join(script_dir, "drone.urdf")

# Define Start Position
START_POS = [0, 0, 1]
START_ORN = p.getQuaternionFromEuler([0, 0, 0])

try:
    droneId = p.loadURDF(drone_urdf_path, START_POS, START_ORN)
except:
    droneId = p.loadURDF("sphere2.urdf", START_POS, globalScaling=0.5)

total_mass = p.getDynamicsInfo(droneId, -1)[0] # Base mass
for i in range(p.getNumJoints(droneId)):
    total_mass += p.getDynamicsInfo(droneId, i)[0] # Child masses

P_GAIN = 0.5
D_GAIN = 0.1
MAX_ROLL = 0.5
HOVER_FORCE = total_mass * 9.8
throttle = HOVER_FORCE
pitch_cmd = 0
roll_cmd = 0
target_roll = 0

# Motor positions (x, y) relative to center
motor_points = [[0.15, 0.15], [-0.15, 0.15], [-0.15, -0.15], [0.15, -0.15]]


def reset_drone():
    """Resets the drone to start position and stops all movement"""
    global throttle
    # 1. Reset Position and Orientation
    p.resetBasePositionAndOrientation(droneId, START_POS, START_ORN)

    # 2. Reset Physics (Velocity) - Critical!
    # If you don't do this, the drone keeps its momentum and flies away again
    p.resetBaseVelocity(droneId, [0, 0, 0], [0, 0, 0])

    # 3. Reset Throttle to Hover
    throttle = HOVER_FORCE
    print(">>> SIMULATION RESET <<<")


print("------------------------------------------------")
print(" CONTROLS:")
print(" [R]          RESET DRONE")
print(" [Left Ctrl]  Decrease Altitude")
print(" [Left Shift] Increase Altitude")
print(" [Arrow Keys] Tilt (Move)")
print("------------------------------------------------")

while True:
    p.stepSimulation()

    # --- INPUT HANDLING ---
    keys = p.getKeyboardEvents()

    # Get current orientation
    pos, orn = p.getBasePositionAndOrientation(droneId)
    current_roll, current_pitch, current_yaw = p.getEulerFromQuaternion(orn)
    lin_vel, ang_vel = p.getBaseVelocity(droneId)
    # 0. RESET BUTTON (Key 'R' is code 114)
    if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
        reset_drone()

    # 1. Throttle (Altitude)
    if p.B3G_SHIFT in keys and (keys[p.B3G_SHIFT] & p.KEY_IS_DOWN):
        throttle += 0.02
    if p.B3G_CONTROL in keys and (keys[p.B3G_CONTROL] & p.KEY_IS_DOWN):
        throttle -= 0.02

    # 2. Pitch (Forward/Back)
    pitch_cmd = 0
    if p.B3G_UP_ARROW in keys and (keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN):
        pitch_cmd = -0.03
    if p.B3G_DOWN_ARROW in keys and (keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN):
        pitch_cmd = 0.03

    # 3. Roll (Left/Right)
    roll_cmd = 0
    target_roll = 0
    if p.B3G_LEFT_ARROW in keys and (keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN):
        target_roll = -0.05
    if p.B3G_RIGHT_ARROW in keys and (keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN):
        target_roll = 0.05

    P_term = P_GAIN * (current_roll-target_roll)
    D_term = D_GAIN * ang_vel[0]
    roll_cmd = P_term + D_term

    print(current_roll)
    roll_angle = math.cos(current_roll)
    if current_roll > MAX_ROLL:
        roll_angle = math.cos(MAX_ROLL)
    elif current_roll < -MAX_ROLL:
        roll_angle = math.cos(-MAX_ROLL)

    vertical_factor = roll_angle * math.cos(current_pitch)
    total_throttle = throttle / vertical_factor
    base_force = total_throttle / 4.0
    # FL, BL, BR, FR
    forces = [
        base_force - roll_cmd + pitch_cmd,  # FL
        base_force - roll_cmd - pitch_cmd,  # BL
        base_force + roll_cmd - pitch_cmd,  # BR
        base_force + roll_cmd + pitch_cmd  # FR
    ]

    # --- APPLY FORCES ---
    for i in range(4):
        p.applyExternalForce(droneId, -1,
                             forceObj=[0, 0, forces[i]],
                             posObj=[motor_points[i][0], motor_points[i][1], 0],
                             flags=p.LINK_FRAME)

    # Camera Update
    pos, orn = p.getBasePositionAndOrientation(droneId)
    p.resetDebugVisualizerCamera(1.5, 270, -35, pos)

    time.sleep(1. / 240.)

p.disconnect()