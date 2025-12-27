import pybullet as p
import math


class CameraManager:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.mode = 0  # 0 = Third Person (Follow), 1 = First Person (Nose)

        # Third Person Settings
        self.follow_dist = 1.5
        self.follow_pitch = -35
        self.follow_yaw = 270  # Behind the drone

    def toggle_mode(self):
        """Switches between camera modes"""
        self.mode = (self.mode + 1) % 2
        print(f"Camera Mode: {'Nose Cam' if self.mode == 1 else 'Follow Cam'}")

    def update(self):
        # Get Drone State
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        euler_angles = p.getEulerFromQuaternion(orn)  # [roll, pitch, yaw] in radians

        if self.mode == 0:
            # --- THIRD PERSON FOLLOW ---
            # Just look at the drone from a fixed distance/angle
            p.resetDebugVisualizerCamera(
                cameraDistance=self.follow_dist,
                cameraYaw=self.follow_yaw,
                cameraPitch=self.follow_pitch,
                cameraTargetPosition=pos
            )

        elif self.mode == 1:
            # --- NOSE CAMERA (Pseudo-FPV) ---

            # Convert Yaw/Pitch to Degrees for PyBullet Camera
            # PyBullet Camera Yaw 0 points North (Y+), Drone Yaw 0 is often X+.
            # We usually need to subtract 90 degrees to align them.
            cam_yaw_deg = math.degrees(euler_angles[2]) - 90
            cam_pitch_deg = math.degrees(-euler_angles[1])

            # We set distance to almost 0 so we are "inside" the target point
            # We set the target point slightly IN FRONT of the drone so we look forward

            # Simple offset math:
            # x = pos[0] + forward_offset * cos(yaw)
            # y = pos[1] + forward_offset * sin(yaw)
            forward_vec = [
                math.cos(euler_angles[2]),
                math.sin(euler_angles[2]),
                0
            ]

            cam_target = [
                pos[0] + (forward_vec[0] * 0.5),  # Look 0.5m ahead
                pos[1] + (forward_vec[1] * 0.5),
                pos[2]
            ]

            p.resetDebugVisualizerCamera(
                cameraDistance=0.1,  # Very close
                cameraYaw=cam_yaw_deg,  # Match Drone Yaw
                cameraPitch=cam_pitch_deg,  # Match Drone Pitch
                cameraTargetPosition=cam_target
            )