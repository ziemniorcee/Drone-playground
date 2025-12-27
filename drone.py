import pybullet as p
import pybullet_data
import time
import os
import math


class DroneController:
    def __init__(self, start_pos, start_orn):
        # --- Constants & Gains ---
        self.total_mass = 0
        self.drag_coeff = 0.1
        self.motor_points = [[0.15, 0.15], [-0.15, 0.15], [-0.15, -0.15], [0.15, -0.15]]

        # PID Gains
        self.P_GAIN = 0.5
        self.D_GAIN = 0.1
        self.P_GAIN_YAW = 0.5
        self.D_GAIN_YAW = 0.1
        self.D_GAIN_THROTTLE = 0.8

        # State Variables
        self.start_pos = start_pos
        self.start_orn = start_orn
        self.target_altitude = 1.0
        self.target_yaw = 0

        # Load the Drone
        self.drone_id = self._load_drone()
        self.hover_force = self.total_mass * 9.8
        self.throttle = self.hover_force

    def _load_drone(self):
        """Loads URDF and calculates mass."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        drone_urdf_path = os.path.join(script_dir, "drone.urdf")

        try:
            d_id = p.loadURDF(drone_urdf_path, self.start_pos, self.start_orn)
        except:
            print("URDF not found, loading sphere placeholder.")
            d_id = p.loadURDF("sphere2.urdf", self.start_pos, globalScaling=0.5)

        # Calculate Mass
        mass = p.getDynamicsInfo(d_id, -1)[0]
        for i in range(p.getNumJoints(d_id)):
            mass += p.getDynamicsInfo(d_id, i)[0]
        self.total_mass = mass
        return d_id

    def reset(self):
        """Resets physical state and targets."""
        p.resetBasePositionAndOrientation(self.drone_id, self.start_pos, self.start_orn)
        p.resetBaseVelocity(self.drone_id, [0, 0, 0], [0, 0, 0])
        self.throttle = self.hover_force
        self.target_altitude = self.start_pos[2]
        self.target_yaw = 0
        print(">>> DRONE RESET <<<")

    def update(self, commands):
        """
        Calculates PID and applies forces.
        Inputs are increments/commands from the user.
        """
        # Update Targets based on Input
        self.target_altitude += commands['alt']
        self.target_yaw += commands['yaw']

        # Get Current State
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        current_roll, current_pitch, current_yaw = p.getEulerFromQuaternion(orn)
        lin_vel_world, ang_vel_world = p.getBaseVelocity(self.drone_id)

        # Convert World Velocity to Body Frame
        invert_pos, invert_orn = p.invertTransform([0, 0, 0], orn)
        lin_vel, _ = p.multiplyTransforms(invert_pos, invert_orn, lin_vel_world, [0, 0, 0, 1])
        ang_vel, _ = p.multiplyTransforms(invert_pos, invert_orn, ang_vel_world, [0, 0, 0, 1])

        # --- PID CALCULATIONS ---

        # 1. Altitude (Throttle)
        P_term_throttle = self.P_GAIN * (self.target_altitude - pos[2])
        D_term_throttle = self.D_GAIN_THROTTLE * (0 - lin_vel_world[2])
        throttle_cmd = P_term_throttle + D_term_throttle

        # 2. Pitch
        P_term_pitch = self.P_GAIN * (current_pitch - commands['pitch'])  # Input acts as target
        D_term_pitch = self.D_GAIN * ang_vel[1]
        pitch_cmd = P_term_pitch + D_term_pitch

        # 3. Yaw
        yaw_error = self.target_yaw - current_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  # Wrap angle
        P_term_yaw = self.P_GAIN_YAW * yaw_error
        D_term_yaw = self.D_GAIN_YAW * (0 - ang_vel[2])
        yaw_cmd = max(min(P_term_yaw + D_term_yaw, 0.4), -0.4)  # Clamp

        # 4. Roll
        P_term_roll = self.P_GAIN * (current_roll - commands['roll'])
        D_term_roll = self.D_GAIN * ang_vel[0]
        roll_cmd = P_term_roll + D_term_roll

        # --- MOTOR MIXING ---
        roll_angle = math.cos(current_roll)
        pitch_angle = math.cos(current_pitch)
        vertical_factor = max(roll_angle * pitch_angle, 0.1)

        total_throttle = (self.throttle + throttle_cmd) / vertical_factor
        base_force = total_throttle / 4.0

        forces = [
            base_force - roll_cmd + pitch_cmd - yaw_cmd,
            base_force - roll_cmd - pitch_cmd + yaw_cmd,
            base_force + roll_cmd - pitch_cmd - yaw_cmd,
            base_force + roll_cmd + pitch_cmd + yaw_cmd
        ]

        # Apply Forces
        for i in range(4):
            p.applyExternalForce(self.drone_id, -1, forceObj=[0, 0, forces[i]],
                                 posObj=[self.motor_points[i][0], self.motor_points[i][1], 0],
                                 flags=p.LINK_FRAME)

            # Apply Drag/Torque
            motor_dir = -1 if i in [0, 2] else 1
            motor_torque = forces[i] * self.drag_coeff * motor_dir
            p.applyExternalTorque(self.drone_id, -1, [0, 0, motor_torque], flags=p.LINK_FRAME)

    def get_pos(self):
        return p.getBasePositionAndOrientation(self.drone_id)[0]

