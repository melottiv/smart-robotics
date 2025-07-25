#! /usr/bin/env python3
import cv2
import numpy as np
import message_filters
import rospy
from skimage import measure
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from pyquaternion import Quaternion as PyQuaternion
from scipy.spatial.distance import euclidean

cam_point = (-0.55, -0.43, 1.80)
table_height = 0.74
table_dist = 0.0
origin = None
model = None
model_orientation = None
pub = None

argv = sys.argv
a_show = '-show' in argv

__all__ = ["start_node"]

# Ingredient color map in BGR
INGREDIENT_COLOR_MAP = {
    "bread":  (102, 153, 204),
    "cheese": (77, 230, 255),
    "meat":   (26, 51, 102),
    "salad":  (51, 153, 51),
    "tomato": (26, 26, 204),
}

# Utility Functions

def get_table_distance(depth):
    global table_dist
    table_dist = np.nanmax(depth)


def get_origin(img):
    global origin
    origin = np.array(img.shape[1::-1]) // 2


# ----------------- LOCALIZATION ----------------- #

def get_color_masks(rgb_img):
    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
    masks = {}

    color_ranges = {
    "cheese": ([17, 150, 200], [37, 255, 255]),  # centro H=27
    "salad":  ([51, 100, 120], [71, 255, 255]),  # centro H=61
    "meat":   ([0, 150,  80], [19, 255, 180]),   # centro H=9
    "bread":  ([5, 100, 160], [25, 200, 255]),   # centro H=15
    "tomato": ([0, 150, 180], [10, 255, 255]),   # centro H=0
    }

    for name, (lower, upper) in color_ranges.items():
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_np, upper_np)
        masks[name] = mask

    return masks

def get_element_distance(region, depth):
    y, x = map(int, region.centroid)
    radius = int(region.axis_major_length)
    slice_box = slice(y - radius, y + radius), slice(x - radius, x + radius)
    l_depth = depth[slice_box]
    if table_dist is not None:
        return table_dist - l_depth.min()
    else:
        return 0.012

theta = 0  

def classify_ingredient_by_color(region, rgb_img):
    y, x = map(int, region.centroid)
    radius = int(region.axis_major_length / 2)
    mask = np.zeros(rgb_img.shape[:2], dtype=np.uint8)
    cv2.circle(mask, (x, y), radius, 255, -1)
    mean_color_bgr = cv2.mean(rgb_img, mask=mask)[:3]
    mean_color_hsv = cv2.cvtColor(np.uint8([[mean_color_bgr]]), cv2.COLOR_BGR2HSV)[0][0]

    min_dist = float("inf")
    best_match = "unknown"

    for name, color_bgr in INGREDIENT_COLOR_MAP.items():
        hsv = cv2.cvtColor(np.uint8([[color_bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        # Pesa meno il valore (luminosità)
        dist = np.linalg.norm([
            mean_color_hsv[0] - hsv[0],   # Hue
            mean_color_hsv[1] - hsv[1],   # Saturation
            0.3 * (mean_color_hsv[2] - hsv[2])  # Value/luminosità pesa meno
        ])
        if dist < min_dist:
            min_dist = dist
            best_match = name

    # Heuristica: se forma molto quadrata e solida, è cheese
    if region.eccentricity < 0.5 and region.solidity > 0.9:
        best_match = "cheese"
        
    print(f"[DEBUG] {region.label}: mean HSV={mean_color_hsv} → {best_match}")
    return best_match



def process_item(item, rgb, depth):
    msg = ModelStates()
    msg.name = item['name']
    region = item['element']
    y, x = map(int, region.centroid)
    element_height = get_element_distance(region, depth)
    xyz = np.array((x, y, element_height / 2 + table_height))
    xyz[:2] /= rgb.shape[1], rgb.shape[0]
    xyz[:2] -= 0.5
    xyz[:2] *= (-0.968, 0.691)
    xyz[:2] *= table_dist / 0.84
    xyz[:2] += cam_point[:2]
    xyz[1] *= 1.01

    dir_z = np.array((0, 0, 1))
    dir_y = np.array((0, 1, 0))
    dir_x = np.array((1, 0, 0))
    theta = 0  
    rot_z = PyQuaternion(axis=dir_z, angle=theta)
    dir_y = rot_z.rotate(dir_y)
    dir_x = rot_z.rotate(dir_x)

    def get_angle(vec, ax):
        vec = np.array(vec)
        if not vec.any():
            return 0
        vec = vec / np.linalg.norm(vec)
        wise = 1 if vec[-1] >= 0 else -1
        dotclamp = max(-1, min(1, np.dot(vec, np.array(ax))))
        return wise * np.arccos(dotclamp)

    rdir_x, rdir_y, rdir_z = dir_x, dir_y, dir_z
    rdir_x[0] *= -1
    rdir_y[0] *= -1
    rdir_z[0] *= -1
    qz1 = PyQuaternion(axis=(0, 0, 1), angle=-get_angle(dir_z[:2], (1, 0)))
    rdir_z = qz1.rotate(dir_z)
    qy2 = PyQuaternion(axis=(0, 1, 0), angle=-get_angle((rdir_z[2], rdir_z[0]), (1, 0)))
    rdir_x = qy2.rotate(qz1.rotate(rdir_x))
    qz3 = PyQuaternion(axis=(0, 0, 1), angle=-get_angle(rdir_x[:2], (1, 0)))

    rot = qz3 * qy2 * qz1
    rot = rot.inverse
    msg.pose = Pose(Point(*xyz), Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w))

    return msg

def merge_close_regions(regions, min_dist=10):
    merged = []
    used = set()
    for i, r1 in enumerate(regions):
        if i in used:
            continue
        for j, r2 in enumerate(regions):
            if j <= i or j in used:
                continue
            if euclidean(r1.centroid, r2.centroid) < min_dist:
                # Merge bounding box (grezzo)
                minr = min(r1.bbox[0], r2.bbox[0])
                minc = min(r1.bbox[1], r2.bbox[1])
                maxr = max(r1.bbox[2], r2.bbox[2])
                maxc = max(r1.bbox[3], r2.bbox[3])
                bbox_mask = np.zeros(labels.shape, dtype=np.uint8)
                bbox_mask[minr:maxr, minc:maxc] = 1
                label = measure.label(bbox_mask, connectivity=2)
                region = measure.regionprops(label)[0]
                merged.append(region)
                used.update([i, j])
                break
        else:
            merged.append(r1)
    return merged

def get_centers_object_color_based(rgb_img):
    masks = get_color_masks(rgb_img)
    positions = []

    for name, mask in masks.items():
        labels = measure.label(mask, connectivity=2)
        regions = measure.regionprops(labels)

        for region in regions:
            if region.area < 50:  # ignora rumore
                continue
            positions.append({'name': name, 'element': region})

    return positions
def process_image(rgb, depth):
    img_draw = rgb.copy()
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    get_table_distance(depth)
    get_origin(gray)

    positions = get_centers_object_color_based(rgb)
    messages = []

    for pos in positions:
        if pos is not None:
            messages.append(process_item(pos, gray, depth))
            y, x = map(int, pos['element'].centroid)
            cv2.circle(img_draw, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(img_draw, pos['name'], (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)

    msg = ModelStates()
    for mess in messages:
        if mess is not None:
            msg.name.append(mess.name)
            msg.pose.append(mess.pose)

    pub.publish(msg)

    if a_show:
        cv2.imshow("vision-results.png", img_draw)
        cv2.waitKey()


def process_callback(image_rgb, image_depth):
    t_start = time.time()
    rgb = CvBridge().imgmsg_to_cv2(image_rgb, "bgr8")
    depth = CvBridge().imgmsg_to_cv2(image_depth, "32FC1")

    process_image(rgb, depth)

    print("Time:", time.time() - t_start)
    rospy.signal_shutdown('0')


def start_node():
    global pub

    print("Starting Node Vision 1.0")

    rospy.init_node('vision')

    print("Subscribing to camera images")
    rgb = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth = message_filters.Subscriber("/camera/depth/image_raw", Image)

    pub = rospy.Publisher("ingredient_detection", ModelStates, queue_size=1,latch=True)

    print("Localization is starting.. ")
    print("(Waiting for images..)", end='\r'), print(end='\033[K')

    synchro = message_filters.TimeSynchronizer([rgb, depth], 1, reset=True)
    synchro.registerCallback(process_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
