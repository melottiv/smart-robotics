#!/usr/bin/python3

import os
import math
import copy
import json
import actionlib
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

MODELS_INFO = {
    "screw": {
        "home": [0.35, -0.5, 0.85],
        "size": [0.0277128, 0.06, 0.024]
    },
    "nut": {
        "home": [0.35, -0.5, 0.85],
        "size": [0.0277128, 0.024, 0.012]
    }
}

SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10


def get_gazebo_model_name(_model_name, _model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.1
    environment_elements = ["ground_plane", "modern_table", "kinect", "robot", "_spawn"]
    for _gazebo_model_name, gazebo_model_pose in zip(models.name, models.pose):
        # Get everything inside a square of side epsilon centered in vision_model_pose
        if _gazebo_model_name not in environment_elements:
            ds = abs(gazebo_model_pose.position.x - _model_pose.position.x) + abs(
                gazebo_model_pose.position.y - _model_pose.position.y)
            if ds <= epsilon:
                return _gazebo_model_name
    raise ValueError(
        f"Model {_model_name} at position {_model_pose.position.x} {_model_pose.position.y} was not found!")


def get_model_name(_gazebo_model_name):
    if _gazebo_model_name.startswith("screw"):
        return "screw"
    elif _gazebo_model_name.startswith('nut'):
        return "nut"
    else:
        return ""


def get_elements_pos(vision=False):
    # get _elements position reading vision topic
    if vision:
        print("Reading from nut_and_screw_detections")
        _elements = rospy.wait_for_message("/nut_and_screw_detections", ModelStates, timeout=None)
    else:
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        _elements = ModelStates()

        for name, pose in zip(models.name, models.pose):
            if "X" not in name:
                continue
            name = get_model_name(name)

            _elements.name.append(name)
            _elements.pose.append(pose)
    print(_elements)
    return [(el_name, el_pose) for el_name, el_pose in zip(_elements.name, _elements.pose)]


def straighten(_model_pose, _gazebo_model_name):
    _x = _model_pose.position.x
    _y = _model_pose.position.y
    _z = _model_pose.position.z
    model_quaternion = PyQuaternion(
        x=_model_pose.orientation.x,
        y=_model_pose.orientation.y,
        z=_model_pose.orientation.z,
        w=_model_pose.orientation.w)

    _model_size = MODELS_INFO[get_model_name(_gazebo_model_name)]["size"]

    """
        Calculate approach quaternion and target quaternion
    """

    facing_direction = (0, 0, 1)
    approach_angle = get_approach_angle(model_quaternion, facing_direction)

    print(f"Lego is facing {facing_direction}")
    print(f"Angle of approaching measures {approach_angle:.2f} deg")

    # Calculate approach quat
    approach_quat = get_approach_quat(facing_direction, approach_angle)

    # Get above the object
    controller.move_to(_x, _y, target_quat=approach_quat)

    """
        Grip the model
    """
    if _gazebo_model_name.startswith('nut'):
        closure = 0.024
    else:
        closure = 0.017
    controller.move_to(z=SURFACE_Z, target_quat=approach_quat)
    close_gripper(_gazebo_model_name, closure)


def close_gripper(_gazebo_model_name, closure=0.):
    set_gripper(0.81 - closure * 10)
    rospy.sleep(0.5)
    # Create dynamic joint
    if _gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = _gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(_gazebo_model_name=None):
    # Destroy dynamic joint
    if _gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = _gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        print(f"Detaching {_gazebo_model_name}")
        detach_srv.call(req)
        set_gripper(0.0)
    else:
        set_gripper(0.0)


def get_approach_quat(facing_direction, approach_angle):
    quater = DEFAULT_QUAT
    if facing_direction == (0, 0, 1):
        pitch_angle = 0
        yaw_angle = 0
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        pitch_angle = + 0.2
        if abs(approach_angle) < math.pi / 2:
            yaw_angle = math.pi / 2
        else:
            yaw_angle = -math.pi / 2
    elif facing_direction == (0, 0, -1):
        pitch_angle = 0
        yaw_angle = 0
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quater = quater * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quater = quater * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quater = PyQuaternion(axis=(0, 0, 1), angle=approach_angle + math.pi / 2) * quater

    return quater


def get_approach_angle(model_quat, facing_direction):  # get gripper approach angle
    if facing_direction == (0, 0, 1):
        return model_quat.yaw_pitch_roll[0] - math.pi / 2  # rotate gripper
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1]))  # get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0)  # sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0)  # cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot)  # get angle between lego z axis and x axis in fixed frame
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi / 2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = -1  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()


if __name__ == "__main__":
    print("Initializing node of kinematics")
    rospy.init_node("send_joints")

    controller = ArmController()

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server()

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    open_gripper()

    print("Waiting for detection of the ninja_models")
    rospy.sleep(0.5)
    elements = get_elements_pos(vision=True)
    elements.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

    for model_name, model_pose in elements:
        open_gripper()
        try:
            element_type = model_name.split('_')[0]
            model_home = MODELS_INFO[element_type]["home"]
            model_size = MODELS_INFO[element_type]["size"]
        except ValueError as e:
            print(f"Model name {model_name} was not recognized!")
            continue

        # Get actual model_name at model xyz coordinates
        try:
            gazebo_model_name = get_gazebo_model_name(model_name, model_pose)
            print(f"{model_name}  ->  {gazebo_model_name}")
        except ValueError as e:
            print(e)
            continue

        # Straighten lego
        straighten(model_pose, gazebo_model_name)
        controller.move(dz=0.15)

        """
            Go to destination
        """
        x, y, z = model_home
        print(f"Moving model {model_name} to {x} {y} {z}")

        controller.move_to(x, y, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=math.pi / 2))
        # Lower the object and release
        controller.move_to(x, y, z)
        # set_model_fixed(gazebo_model_name)
        open_gripper(gazebo_model_name)
        controller.move(dz=0.15)

        if controller.gripper_pose[0][1] > -0.3 and controller.gripper_pose[0][0] > 0:
            controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    print("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rospy.sleep(0.4)
