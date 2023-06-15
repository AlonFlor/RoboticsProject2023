import pybullet as p

import file_handling
import pybullet_utilities as p_utils
import os
import numpy as np

#physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)
os.mkdir(test_dir)

push_distance = 0.15


view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)
point_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.005)


mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []
mobile_object_COM_candidates = []
mobile_object_axes = []
mobile_object_axes_origins = []
mobile_object_COM_candidate_probabilities = []

p_utils.open_saved_scene(os.path.join("scenes","scene_COM_overlay_one_object.csv"), test_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list)


def get_candidate_COMs_and_object_axes(mobile_object_index, number_of_points_axis_0, number_of_points_axis_1):
    bounding_points = file_handling.read_csv_file(os.path.join("object models",mobile_object_types[mobile_object_index],"precomputed_bounding_points.csv"),[float, float, float])
    COM_ground_truth = np.array(p.getDynamicsInfo(mobile_object_IDs[mobile_object_index],-1)[3])
    position, orientation = p.getBasePositionAndOrientation(mobile_object_IDs[mobile_object_index])
    position = np.array(position)
    orientation = np.array(orientation)

    bounding_points_world_coords = []
    for point in bounding_points:
        point_array = np.array(point)
        new_point = p_utils.rotate_vector(point_array - COM_ground_truth, orientation) + position
        bounding_points_world_coords.append(new_point)

    #get xy plane
    #need bounding points corresponding to xy plane
    #perhaps these can be found by comparing xy distances of all transformed bounding points, and finding the pair with the smallest distance
    #that pair has one point that needs to be removed, it is the one father away on the z-axis from the two points not in the pair
    min_dist = 100.
    pair = (0,1)
    for i in np.arange(len(bounding_points_world_coords)):
        for j in np.arange(i+1, len(bounding_points_world_coords)):
            dist = np.linalg.norm(bounding_points_world_coords[i][:2] - bounding_points_world_coords[j][:2])
            if min_dist > dist:
                min_dist = dist
                pair = (i,j)
    dists_first = 0.
    dists_second = 0.
    point_to_eliminate_index = 0
    for i in np.arange(len(bounding_points_world_coords)):
        if i in pair:
            continue
        dists_first+= np.abs(bounding_points_world_coords[i][2] - bounding_points_world_coords[pair[0]][2])
        dists_second+= np.abs(bounding_points_world_coords[i][2] - bounding_points_world_coords[pair[1]][2])
    if dists_first > dists_second:
        point_to_eliminate_index = pair[0]
    else:
        point_to_eliminate_index = pair[1]
    bounding_points_world_coords.pop(point_to_eliminate_index)

    #once a bounding plane has been defined, then need to create array of points corresponding to potential COMs. Points are defined relative to object
    axis_0_raw_values = np.linspace(0., 1., number_of_points_axis_0+2)[1:-1]
    axis_1_raw_values = np.linspace(0., 1., number_of_points_axis_1+2)[1:-1]
    axis_0_values = np.array([bounding_points_world_coords[0] * value + bounding_points_world_coords[1] * (1. - value) for value in axis_0_raw_values])
    axis_1_values = np.array([bounding_points_world_coords[0] * value + bounding_points_world_coords[2] * (1. - value) for value in axis_1_raw_values])

    plane_origin = bounding_points_world_coords[0]

    #get the COM candidate locs in object space
    COM_candidates_locs = []
    orientation_opposite = np.array([orientation[0], orientation[1], orientation[2], -orientation[3]])
    for value_0 in axis_0_values:
        for value_1 in axis_1_values:
            loc_world_space = value_0 + value_1 - plane_origin
            loc_object_space = p_utils.rotate_vector(loc_world_space - position, orientation_opposite)
            COM_candidates_locs.append(loc_object_space)

    #get the object's axes in object space
    axis_0 = bounding_points_world_coords[1] - bounding_points_world_coords[0]
    axis_1 = bounding_points_world_coords[2] - bounding_points_world_coords[0]
    axis_0 = axis_0 / np.linalg.norm(axis_0)
    axis_1 = axis_1 / np.linalg.norm(axis_1)
    axis_0_object_space = p_utils.rotate_vector(axis_0, orientation_opposite)
    axis_1_object_space = p_utils.rotate_vector(axis_1, orientation_opposite)

    origin_of_axes = p_utils.rotate_vector(plane_origin - position, orientation_opposite)

    return COM_candidates_locs, (axis_0_object_space, axis_1_object_space), origin_of_axes


def get_world_space_point(point, position, orientation):
    return p_utils.rotate_vector(point, orientation)+position

def display_COM_candidates(mobile_object_index):
    position, orientation = p.getBasePositionAndOrientation(mobile_object_IDs[mobile_object_index])
    position = np.array(position)
    orientation = np.array(orientation)
    displayed_shapes_IDs = []
    max_prob = max(mobile_object_COM_candidate_probabilities[mobile_object_index])
    for i,value in enumerate(mobile_object_COM_candidates[mobile_object_index]):
        loc = get_world_space_point(value, position, orientation)
        point_id = p.createMultiBody(baseVisualShapeIndex=point_visual_shape, basePosition=(loc[0], loc[1], 0.15))
        color_val = mobile_object_COM_candidate_probabilities[mobile_object_index][i]/max_prob
        p.changeVisualShape(point_id, -1, rgbaColor=(color_val,color_val,color_val, 1.))
        displayed_shapes_IDs.append(point_id)
    for point_id in displayed_shapes_IDs:
        p.removeBody(point_id)
    displayed_shapes_IDs.clear()


def get_pusher_start_and_direction_points(mobile_object_index, axis_index):
    position, orientation = p.getBasePositionAndOrientation(mobile_object_IDs[mobile_object_index])
    position = np.array(position)
    orientation = np.array(orientation)

    #get average COM
    avg_point = np.array([0.,0.])
    for i in np.arange(len(mobile_object_COM_candidates[mobile_object_index])):
        candidate = mobile_object_COM_candidates[mobile_object_index][i]
        candidate_world_coords = get_world_space_point(candidate, position, orientation)
        avg_point += mobile_object_COM_candidate_probabilities[mobile_object_index][i]*candidate_world_coords[:2]

    #get start point
    axis_of_start_point = mobile_object_axes[mobile_object_index][axis_index]
    axis_of_start_point_world_space = p_utils.rotate_vector(axis_of_start_point, orientation)[:2]
    object_origin_world_coords = get_world_space_point(mobile_object_axes_origins[mobile_object_index], position, orientation)[:2]
    start_point = object_origin_world_coords + np.dot(avg_point - object_origin_world_coords, axis_of_start_point_world_space) * axis_of_start_point_world_space

    return start_point, avg_point


def calculate_probability_for_a_single_candidate_COM(mobile_object_index, candidate_COM_index, original_position, original_orientation, pusher_start, pusher_dir, angular_displacement):
    candidate_COM = get_world_space_point(mobile_object_COM_candidates[mobile_object_index][candidate_COM_index], original_position, original_orientation)
    torque = np.cross(pusher_start-candidate_COM,pusher_dir)[2]
    factor = angular_displacement/torque
    #print(factor)
    if factor <= 0:
        return 0.
    return 1.#factor


#initialize COM candidates and COM axes
for i in np.arange(len(mobile_object_IDs)):
    COM_candidates, axes, axes_origin = get_candidate_COMs_and_object_axes(i, 10, 10)
    mobile_object_COM_candidates.append(COM_candidates)
    mobile_object_axes.append(axes)
    mobile_object_axes_origins.append(axes_origin)
    prob = 1. / len(COM_candidates)
    mobile_object_COM_candidate_probabilities.append([prob for i in COM_candidates])


display_COM_candidates(0)

cylinderID = p_utils.create_cylinder(0.015 / 2, 0.05)
cylinder_height_offset = np.array([0., 0., 0.02])

new_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(mobile_object_IDs[0])[1])[2]

for round_index in range(5):
    old_angle = new_angle + 0.

    orig_position, orig_orientation = p.getBasePositionAndOrientation(mobile_object_IDs[0])
    orig_position = np.array(orig_position)
    orig_orientation = np.array(orig_orientation)

    #figure out where to push next
    point_1, point_2 = get_pusher_start_and_direction_points(0, round_index%2)
    point_1 = np.array([point_1[0],point_1[1],0.]) + cylinder_height_offset
    point_2 = np.array([point_2[0],point_2[1],0.]) + cylinder_height_offset

    direction = point_2 - point_1
    direction_normalized = direction / np.linalg.norm(direction)
    point_2 = push_distance * direction_normalized + point_1

    #push
    p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))
    p_utils.push(point_2, cylinderID, dt, time_out=200.)
    p_utils.let_time_pass(cylinderID, dt,mobile_object_IDs)

    #get the new angle and angular displacement
    new_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(mobile_object_IDs[0])[1])[2]
    angular_displacement = new_angle - old_angle
    if angular_displacement < -3 * np.pi / 4:
        angular_displacement += 2 * np.pi
    elif angular_displacement > 3 * np.pi / 4:
        angular_displacement -= 2 * np.pi

    print("angular_displacement",angular_displacement)

    #recalculate probabilities
    prob_sum = 0.
    for candidate_COM_index in np.arange(len(mobile_object_COM_candidate_probabilities[0])):
        if mobile_object_COM_candidate_probabilities[0][candidate_COM_index] > 0:
            mobile_object_COM_candidate_probabilities[0][candidate_COM_index] = \
                calculate_probability_for_a_single_candidate_COM(0, candidate_COM_index, orig_position, orig_orientation, point_1, direction_normalized, angular_displacement)
            prob_sum += mobile_object_COM_candidate_probabilities[0][candidate_COM_index]

    for candidate_COM_index in np.arange(len(mobile_object_COM_candidate_probabilities[0])):
        mobile_object_COM_candidate_probabilities[0][candidate_COM_index] /= prob_sum #normalize

    #write down the centers of mass
    COM_ground_truth = np.array(p.getDynamicsInfo(mobile_object_IDs[0], -1)[3])
    COM_estimate = np.array([0., 0., 0.])
    for candidate_COM_index in np.arange(len(mobile_object_COM_candidate_probabilities[0])):
        COM_estimate += mobile_object_COM_candidate_probabilities[0][candidate_COM_index] * mobile_object_COM_candidates[0][candidate_COM_index]
    print("COM_ground_truth object coords",COM_ground_truth)
    print("COM estimated object coords -",COM_estimate)

    display_COM_candidates(0)


p.disconnect()

'''planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)
boxID, _ = p_utils.load_object("cracker_box", test_dir, (-0.01,-0.01,0.08))

mobile_object_IDs.append(boxID)
mobile_object_types.append("cracker_box")

p.resetBasePositionAndOrientation(boxID, (0.,0.,1.), p.getQuaternionFromEuler((0.,np.pi/2, 0.)))


import time

time_amount = 1.
count=0
while time_amount > 0:
    time_val = count * dt
    count += 1

    p.stepSimulation()

    time.sleep(dt)
    time_amount -= dt

held_fixed_list = [False for item in mobile_object_types]
p_utils.save_scene_no_bin(os.path.join(test_dir,"scene.csv"), mobile_object_IDs, mobile_object_types, held_fixed_list)

#print an image of the scene
p_utils.print_image(view_matrix, proj_matrix, test_dir, 0)


p.disconnect()'''
