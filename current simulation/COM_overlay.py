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



view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)


mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []

p_utils.open_saved_scene(os.path.join("scenes","scene_COM_overlay_one_object.csv"), test_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list)


def get_candidate_COMs(mobile_object_index):
    bounding_points = file_handling.read_csv_file(os.path.join("object models",mobile_object_types[mobile_object_index],"precomputed_bounding_points.csv"),[float, float, float])
    print(bounding_points)
    COM_ground_truth = np.array(p.getDynamicsInfo(mobile_object_IDs[mobile_object_index],-1)[3])
    position, orientation = p.getBasePositionAndOrientation(mobile_object_IDs[mobile_object_index])
    position = np.array(position)
    orientation = np.array(orientation)

    bounding_points_transformed = []
    for point in bounding_points:
        point_array = np.array(point)
        new_point = p_utils.rotate_vector(point_array - COM_ground_truth, orientation) + position
        bounding_points_transformed.append(new_point)
    print(bounding_points_transformed)

    #get xy plane
    #need bounding points corresponding to xy plane
    #perhaps these can be found by comparing xy distances of all transformed bounding points, and finding the pair with the smallest distance
    #that pair has one point that needs to be removed, it is the one father away on the z-axis from the two points not in the pair
    min_dist = 100.
    pair = (0,1)
    for i in np.arange(len(bounding_points_transformed)):
        for j in np.arange(i+1,len(bounding_points_transformed)):
            dist = np.linalg.norm(bounding_points_transformed[i][:2] - bounding_points_transformed[j][:2])
            if min_dist > dist:
                min_dist = dist
                pair = (i,j)
    dists_first = 0.
    dists_second = 0.
    point_to_eliminate_index = 0
    for i in np.arange(len(bounding_points_transformed)):
        if i in pair:
            continue
        dists_first+= np.abs(bounding_points_transformed[i][2] - bounding_points_transformed[pair[0]][2])
        dists_second+= np.abs(bounding_points_transformed[i][2] - bounding_points_transformed[pair[1]][2])
    if dists_first > dists_second:
        point_to_eliminate_index = pair[0]
    else:
        point_to_eliminate_index = pair[1]
    bounding_points_transformed.pop(point_to_eliminate_index)


    #once a bounding plane has been defined, then need to create array of points corresponding to potential COMs. Points are defined relative to object
    number_of_points_axis_0 = 10
    number_of_points_axis_1 = 10
    axis_0_raw_values = np.linspace(0., 1., number_of_points_axis_0+2)[1:-1]
    axis_1_raw_values = np.linspace(0., 1., number_of_points_axis_1+2)[1:-1]
    print("axis_0_raw_values",axis_0_raw_values)
    print("axis_1_raw_values",axis_1_raw_values)
    axis_0_values = np.array([bounding_points_transformed[0]*value + bounding_points_transformed[1]*(1.-value) for value in axis_0_raw_values])
    axis_1_values = np.array([bounding_points_transformed[0]*value + bounding_points_transformed[2]*(1.-value) for value in axis_1_raw_values])

    plane_origin = bounding_points_transformed[0]

    COM_candidates_locs = []
    orientation_opposite = np.array([orientation[0], orientation[1], orientation[2], -orientation[3]])
    for value_0 in axis_0_values:
        for value_1 in axis_1_values:
            loc_world_space = value_0 + value_1 - plane_origin
            loc_object_space = p_utils.rotate_vector(loc_world_space - position, orientation_opposite)
            COM_candidates_locs.append(loc_object_space)

    return COM_candidates_locs


COM_candidates_locs = get_candidate_COMs(0)
point_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.005)

position, orientation = p.getBasePositionAndOrientation(mobile_object_IDs[0])
position = np.array(position)
orientation = np.array(orientation)
for value in COM_candidates_locs:
    loc = p_utils.rotate_vector(value, orientation)+position
    point_id = p.createMultiBody(baseVisualShapeIndex=point_visual_shape, basePosition=(loc[0], loc[1], 0.15))


import time

time_amount = 1.
count=0
while time_amount > 0:
    time_val = count * dt
    count += 1

    p.stepSimulation()

    time.sleep(dt)
    time_amount -= dt


position, orientation = p.getBasePositionAndOrientation(mobile_object_IDs[0])
position = np.array(position)
orientation = np.array(orientation)
for value in COM_candidates_locs:
    loc = p_utils.rotate_vector(value, orientation)+position
    point_id = p.createMultiBody(baseVisualShapeIndex=point_visual_shape, basePosition=(loc[0], loc[1], 0.15))


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
