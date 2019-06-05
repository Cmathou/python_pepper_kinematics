#!/usr/bin/env python
# coding: utf-8

import time
import cv2
import pybullet
import pybullet_data
from qibullet import PepperVirtual
from qibullet import SimulationManager
import pepper_kinematics as pk


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1)
    pepper.setAngles(pk.right_arm_tags, pk.right_arm_work_pose, 1.0)
    time.sleep(1.0)
    pepper.setAngles(pk.left_arm_tags, pk.left_arm_work_pose, 1.0)
    time.sleep(1.0)


    current_angles = pepper.getAnglesPosition(pk.right_arm_tags)
    current_position, current_orientation = pk.right_arm_get_position(current_angles)
    print current_position
    target_position = current_position
    target_position[0] = target_position[0] - 0.10# 5 cm toward left
    target_position[1] = target_position[1] + 0.20 # 5 cm toward left
    target_position[2] = target_position[2] + 0.05 # 5 cm toward left
    target_orientation = current_orientation # This is not supported yet
    
    target_angles = pk.right_arm_set_position(current_angles, target_position, target_orientation, 0.01)
    if target_angles.any():
        pepper.setAngles(pk.right_arm_tags, target_angles.tolist(), 1.0)
    time.sleep(3)

    current_angles = pepper.getAnglesPosition(pk.left_arm_tags)
    current_position, current_orientation = pk.left_arm_get_position(current_angles)
    print current_position
    target_position = current_position
    target_position[0] = target_position[0] - 0.10# 5 cm toward left
    target_position[1] = target_position[1] - 0.10 # 5 cm toward left
    target_position[2] = target_position[2] + 0.10 # 5 cm toward left
    target_orientation = current_orientation # This is not supported yet
    
    target_angles = pk.left_arm_set_position(current_angles, target_position, target_orientation, 0.01)
    if target_angles.any():
        pepper.setAngles(pk.left_arm_tags, target_angles.tolist(), 1.0)
    time.sleep(3)


    time.sleep(10)

if __name__ == "__main__":
    main()