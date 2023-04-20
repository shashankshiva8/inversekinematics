#!/usr/bin/env python

import stretch_body.robot
import time

import ikpy.urdf.utils
import pathlib
import stretch_body.hello_utils as hu
from IPython import display
from IPython.display import SVG
import ipywidgets as widgets

import urdfpy
import numpy as np

import ikpy.chain


def move_to_configuration(tool, q):
    if tool == 'tool_stretch_gripper':
        q_base = q[1]
        q_lift = q[3]
        q_arm = q[5] + q[6] + q[7] + q[8]
        q_yaw = q[9]
        robot.base.translate_by(q_base)
        robot.lift.move_to(q_lift)
        robot.arm.move_to(q_arm)
        robot.end_of_arm.move_to('wrist_yaw', q_yaw)
        robot.push_command()


def get_current_configuration(tool):

    def bound_range(name, value):
        names = [l.name for l in chain.links]
        index = names.index(name)
        bounds = chain.links[index].bounds
        return min(max(value, bounds[0]), bounds[1])

    if tool == 'tool_stretch_gripper':
        q_base = 0.0
        q_lift = bound_range('joint_lift', robot.lift.status['pos'])
        q_arml = bound_range('joint_arm_l0', robot.arm.status['pos'] / 4.0)
        q_yaw = bound_range('joint_wrist_yaw', robot.end_of_arm.status['wrist_yaw']['pos'])
        return [0.0, q_base, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, 0.0]


robot = stretch_body.robot.Robot()
robot.startup()


urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
tree = ikpy.urdf.utils.get_urdf_tree(urdf_path, "base_link")[0]
# # display.display_png(tree)

print(robot.end_of_arm.name)

original_urdf = urdfpy.URDF.load(urdf_path)
# print(f"name: {original_urdf.name}")
# print(f"num links: {len(original_urdf.links)}")
# print(f"num joints: {len(original_urdf.joints)}")

modified_urdf = original_urdf.copy()
names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'laser', 'respeaker_base']
links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
for lr in links_to_remove:
    modified_urdf._links.remove(lr)
names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'joint_laser', 'joint_respeaker']
joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
for jr in joints_to_remove:
    modified_urdf._joints.remove(jr)
# # print(f"name: {modified_urdf.name}")
# # print(f"num links: {len(modified_urdf.links)}")
# # print(f"num joints: {len(modified_urdf.joints)}")


# joint_base_translation = urdfpy.Joint(name='joint_base_translation',
#                                       parent='base_link',
#                                       child='link_base_translation',
#                                       joint_type='prismatic',
#                                       axis=np.array([1.0, 0.0, 0.0]),
#                                       origin=np.eye(4, dtype=np.float64),
#                                       limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.0, upper=1.0)))
# modified_urdf._joints.append(joint_base_translation)
# link_base_translation = urdfpy.Link(name='link_base_translation',
#                                     inertial=None,
#                                     visuals=None,
#                                     collisions=None)
# modified_urdf._links.append(link_base_translation)
#
# # amend the chain
# for j in modified_urdf._joints:
#     if j.name == 'joint_mast':
#         j.parent = 'link_base_translation'

# print(f"name: {modified_urdf.name}")
# print(f"num links: {len(modified_urdf.links)}")
# print(f"num joints: {len(modified_urdf.joints)}")

iktuturdf_path = "/tmp/iktutorial/stretch.urdf"
modified_urdf.save(iktuturdf_path)

tree = ikpy.urdf.utils.get_urdf_tree(iktuturdf_path, "base_link")[0]
display.display_png(tree)

chain = ikpy.chain.Chain.from_urdf_file(iktuturdf_path)
# print(chain.links)


# get_current_configuration(tool=robot.end_of_arm.name)
print(get_current_configuration(tool=robot.end_of_arm.name))


target_point = [-0.03, -0, 0.654]


target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, -np.pi/2)
pretarget_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)

q_init = get_current_configuration(tool=robot.end_of_arm.name)
q_mid = chain.inverse_kinematics(target_point, pretarget_orientation, orientation_mode='all', initial_position=q_init)
q_soln = chain.inverse_kinematics(target_point, target_orientation, orientation_mode='all', initial_position=q_mid)

# print(np.linalg.norm(chain.forward_kinematics(q_soln)[:3, 3] - target_point))

move_to_configuration(tool=robot.end_of_arm.name, q=q_soln)