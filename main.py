import pybullet as p
import pybullet_data
import time
import os
import math

from drone import DroneController
from input_camera import CameraManager
from input_manager import InputManager


def main():
    p.connect(p.GUI, options='--width=1920 --height=1080')
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    # Initialize Drone Class
    START_POS = [0, 0, 1]
    START_ORN = p.getQuaternionFromEuler([0, 0, 0])
    drone = DroneController(START_POS, START_ORN)
    camera = CameraManager(drone.drone_id)
    print("------------------------------------------------")
    print(" CONTROLS: [R] Reset | [Arrows] Move | [Ctrl/Shift] Altitude | [A/D] Yaw")
    print("------------------------------------------------")

    timeStep = 1. / 240.
    controls = InputManager()
    p.setTimeStep(timeStep)

    while True:
        start_time = time.time()

        cmds = controls.get_commands()

        if cmds['reset']:
            drone.reset()

        if cmds['camera_toggle']:
            camera.toggle_mode()

        drone.update(cmds)

        p.stepSimulation()
        camera.update()


        elapsed = time.time() - start_time
        if elapsed < (1. / 240.):
            time.sleep((1. / 240.) - elapsed)


if __name__ == "__main__":
    main()
