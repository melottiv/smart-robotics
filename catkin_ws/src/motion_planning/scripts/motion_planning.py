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
from collections import defaultdict
from openai import OpenAI
import json



PKG_PATH = os.path.dirname(os.path.abspath(__file__))

PILING_LOCATION=[0.35, -0.5, 0.774]

MODEL_SIZES = {
    'bread':    (0.07, 0.07, 0.015),     
    'tomato':   (0.04, 0.04, 0.002),      
    'cheese':   (0.06, 0.06, 0.003),    
    'meat':     (0.06, 0.06, 0.007),     
    'salad':    (0.06, 0.06, 0.005)      
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
    ingredientList = ["bread", "cheese", "meat", "tomato", "salad"]
    for ingredient in ingredientList:
        if _gazebo_model_name.startswith(ingredient):
            return ingredient
    return ""


def get_elements_pos(vision=False):
    # get _elements position reading vision topic
    if vision:
        print("Reading from ingredient_detection")
        _elements = rospy.wait_for_message("/ingredient_detection", ModelStates, timeout=None)
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

    _model_size = MODEL_SIZES[get_model_name(_gazebo_model_name)]

    """
        Calculate approach quaternion and target quaternion
    """

    facing_direction = (0, 0, 1)
    approach_angle = get_approach_angle(model_quaternion, facing_direction)

    print(f"Ingredient is facing {facing_direction}")
    print(f"Angle of approaching measures {approach_angle:.2f} deg")

    # Calculate approach quat
    approach_quat = get_approach_quat(facing_direction, approach_angle)

    # Get above the object
    controller.move_to(_x, _y, target_quat=approach_quat)

    diameter = max(_model_size[0], _model_size[1])
    CLOSURE_FACTOR = 1  # da regolare con test empirici
    closure = diameter * CLOSURE_FACTOR

    # Afferraggio
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

def burger_sort(elements):
    ingredient_order = ['bread', 'salad','meat', 'cheese', 'tomato', 'bread']

    ingredient_map = defaultdict(list)

    for name, pose in elements:
        if name.startswith('bread'):
            ingredient_map['bread'].append((name, pose))
        elif name.startswith('meat'):
            ingredient_map['meat'].append((name, pose))
        elif name.startswith('salad'):
            ingredient_map['salad'].append((name, pose))
        elif name.startswith('cheese'):
            ingredient_map['cheese'].append((name, pose))
        elif name.startswith('tomato'):
            ingredient_map['tomato'].append((name, pose))
        else:
            print(f"Unknown ingredient: {name}")

    ordered_models = []
    for ingredient in ingredient_order:
        if ingredient_map[ingredient]:
            ordered_models.append(ingredient_map[ingredient].pop(0))
        else:
            print(f"Ingredient: {ingredient} is missing")
    return ordered_models


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

def set_model_fixed(model_name):
    req=AttachRequest()
    req.model_name_1= model_name
    req.link_name_1="link"
    req.model_name_2=last_gazebo_model_name
    req.link_name_2="link"
    attach_srv.call(req)

def get_ingredient_list_from_user():
    print("\nHai a disposizione questi ingredienti:")
    print("  bread, meat, cheese, tomato, salad")
    print("Scrivi la lista separata da virgole, es:")
    print("  bread, meat, meat, cheese, bread")
    user_input = input("\nIngredienti del panino (dal basso verso l’alto): ")

    # Pulisci e normalizza
    raw_ingredients = [item.strip().lower() for item in user_input.split(",")]

    valid_ingredients = {"bread", "meat", "cheese", "tomato", "salad"}
    filtered_ingredients = []

    for ing in raw_ingredients:
        if ing in valid_ingredients:
            filtered_ingredients.append(ing)
        else:
            print(f" Ingrediente non valido ignorato: {ing}")

    if not filtered_ingredients:
        print(" Nessun ingrediente valido fornito. Uso fallback.")
        return ['bread', 'meat', 'cheese', 'bread']

    return filtered_ingredients



def get_ingredient_list_from_gpt():
    prompt = (
        "Sei un assistente per un robot che costruisce panini.\n"
        "Ingredienti disponibili: bread, meat, cheese, tomato, salad.\n"
        "L'utente può chiedere ingredienti anche in quantità multiple (es: doppia carne, extra formaggio).\n"
        "Restituisci **solo** una lista JSON ordinata con i nomi degli ingredienti (in inglese), "
        "in ordine dal basso verso l’alto del panino, iniziando e finendo con il 'bread'.\n"
        "Esempio output: [\"bread\", \"meat\", \"meat\", \"cheese\", \"tomato\", \"bread\"]\n\n"
        "Frase dell’utente: "
    )

    user_input = input("Cosa vuoi nel tuo panino? (ingredienti disponibili: insalata, carne, formaggio e pomodoro): ")
    full_prompt = prompt + user_input

    response = client.chat.completions.create(
        model="gpt-3.5-turbo",  
        messages=[
            {"role": "user", "content": full_prompt}
        ],
        temperature=0.3
    )

    try:
        content = response.choices[0].message.content
        ingredient_list = json.loads(content)
        print(f"\nIngredienti scelti: {ingredient_list}")
        return ingredient_list
    except Exception as e:
        print("Errore interpretando la risposta GPT, uso fallback:", e)
        return ['bread', 'meat', 'cheese', 'tomato', 'bread']


def burger_sort_gpt(elements, ingredient_order):
    ingredient_map = defaultdict(list)

    for name, pose in elements:
        ingredient = get_model_name(name)
        if ingredient in MODEL_SIZES:
            ingredient_map[ingredient].append((name, pose))
        else:
            print(f"Ingrediente sconosciuto: {name}")

    ordered_models = []
    for ingredient in ingredient_order:
        if ingredient_map[ingredient]:
            ordered_models.append(ingredient_map[ingredient].pop(0))
        else:
            print(f"Ingrediente mancante: {ingredient}")
    return ordered_models




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

    user_ingredients = get_ingredient_list_from_user()
    ordered_models = burger_sort_gpt(elements, user_ingredients)

    x, y = PILING_LOCATION[0], PILING_LOCATION[1]
    cumulative_height = 0.0
    last_gazebo_model_name= "ground_plane"

    for model_name, model_pose in ordered_models:
        open_gripper()
        try:
            element_type = model_name.split('_')[0]
            model_size = MODEL_SIZES[element_type]
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
        controller.move(dz=0.10)

        """
            Go to destination
        """
                
        # Calcola la z corretta in base all'altezza cumulata
        release_z = SURFACE_Z + cumulative_height + (model_size[2] / 2.0) 

        # Orientazione: gripper orizzontale
        approach_quat = DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=math.pi / 2)

        print(f"Moving model {model_name} to {x:.3f} {y:.3f} {release_z:.3f}")

        # Vai sopra al punto di rilascio
        controller.move_to(x, y, release_z + 0.05, target_quat=approach_quat)

        # Abbassati per rilasciare con precisione
        controller.move_to(x, y, release_z, target_quat=approach_quat)

        # Rilascia il pezzo

        open_gripper(gazebo_model_name)
        rospy.sleep(0.5)
        
        set_model_fixed(gazebo_model_name)
        print(f"attached {gazebo_model_name} to {last_gazebo_model_name}")
        last_gazebo_model_name= gazebo_model_name


        # Sollevati
        controller.move(dz=0.10)

        # Aggiorna l'altezza cumulata
        cumulative_height += model_size[2]


        if controller.gripper_pose[0][1] > -0.3 and controller.gripper_pose[0][0] > 0:
            controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    print("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rospy.sleep(0.4)
