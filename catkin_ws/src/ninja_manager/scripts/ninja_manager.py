#!/usr/bin/python3
import argparse
import os
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import rospy
import rosservice
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
import random
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest


path = rospkg.RosPack().get_path("ninja_manager")

# DEFAULT PARAMETERS
package_name = "ninja_manager"
spawn_name = '_spawn'
spawn_pos = (-0.38, -0.35, 0.74)
spawn_dim = (0.25, 0.2)
min_space = 0.010
min_distance = 0.15

ingredientDict = {
    'bread':    (0, (0.06, 0.06, 0.025)),     
    'tomato':   (1, (0.04, 0.04, 0.01)),      
    'cheese':   (2, (0.06, 0.06, 0.003)),    
    'meat':     (3, (0.05, 0.05, 0.015)),     
    'salad':    (4, (0.06, 0.06, 0.005))       
}


ingredientList = list(ingredientDict.keys())
counters = [0 for _ in ingredientList]
spawned_ingredients = []


# get model path
def get_model_path(model):
    pkg_path = rospkg.RosPack().get_path(package_name)
    return f'{pkg_path}/ingredients_models/{model}/model.sdf'


# set position 
def random_pose(model_type):
    _, dim = ingredientDict[model_type]
    spawn_x = spawn_dim[0]
    spawn_y = spawn_dim[1]
    rot_x = 0
    rot_y = 0
    rot_z = random.uniform(-3.14, 3.14)
    point_x = random.uniform(-spawn_x, spawn_x)
    point_y = random.uniform(-spawn_y, spawn_y)
    point_z = dim[2] / 2

    rot = Quaternion(*quaternion_from_euler(rot_x, rot_y, rot_z))
    point = Point(point_x, point_y, point_z)
    return Pose(point, rot), dim[0], dim[1]


class PoseError(Exception):
    pass


# function to get a valid pose
def get_valid_pose(model_type):
    trys = 1000
    valid = False
    while not valid:
        pos, dim1, dim2 = random_pose(model_type)
        radius = np.sqrt((dim1**2 + dim2**2)) / 2
        valid = True
        for ing in spawned_ingredients:
            point = ing[2].position
            r2 = ing[3]
            min_dist = max(radius + r2 + min_space, min_distance)
            if (point.x - pos.position.x)**2 + (point.y - pos.position.y)**2 < min_dist**2:
                valid = False
                trys -= 1
                if trys == 0:
                    raise PoseError("No space available in spawn area")
                break
    return pos, radius


# function to spawn model
def spawn_model(model, pos, name=None, ref_frame='world'):
    if name is None:
        name = model
    model_xml = open(get_model_path(model), 'r').read()
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    return spawn_model_client(model_name=name,
                              model_xml=model_xml,
                              robot_namespace='/food',
                              initial_pose=pos,
                              reference_frame=ref_frame)



# Lista ordinata degli ingredienti da spawnare ogni volta
fixed_ingredient_order = ['bread', 'meat', 'cheese', 'salad', 'tomato','bread']


def spawn_all_ingredients():
    for model_type in fixed_ingredient_order:
        idx = ingredientDict[model_type][0]
        name = f'{model_type}_{str(time.time_ns())[-9:]}'
        pos, radius = get_valid_pose(model_type)
        spawn_model(model_type, pos, name, spawn_name)
        spawned_ingredients.append((name, model_type, pos, radius))
        counters[idx] += 1

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = model_name
        delete_srv(req)
        rospy.loginfo(f"Deleted model: {model_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to delete model {model_name}: {e}")

# main function setup area and level manager
def set_up_area():
    rospy.init_node('ingredient_spawner')

    models = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    environment_elements = ["ground_plane", "modern_table", "kinect", "robot", "_spawn"]
    spawn_area_exists = any(m.startswith('_spawn') for m in models.name)

    for model in models.name:
        if model not in environment_elements and any(model.startswith(i) for i in ingredientList):
            delete_model(model)

    if not spawn_area_exists:
        spawn_model(spawn_name, Pose(Point(*spawn_pos), Quaternion(0, 0, 0, 1)))

    try:
        spawn_all_ingredients()   
    except PoseError:
        print("[Error]: no space in spawning area")

    print(f"Added {len(spawned_ingredients)} ingredient(s)")


if __name__ == '__main__':

    try:
        if '/gazebo/spawn_sdf_model' not in rosservice.get_service_list():
            print("Waiting for gazebo service..")
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model(spawn_name, Pose(Point(*spawn_pos), Quaternion(0, 0, 0, 1)))
        set_up_area()
        print("All done. Ready to start.")
    except Exception as e:
        print(f"Exception: {e}")
