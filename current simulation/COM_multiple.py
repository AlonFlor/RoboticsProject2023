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


#define pushing data
push_distance = 0.15
cylinder_height_offset = np.array([0., 0., 0.02])

point_1 = np.array([0.3,0.,0.]) + cylinder_height_offset
point_2 = np.array([-0.3,0.,0.]) + cylinder_height_offset

direction = point_2 - point_1
direction_normalized = direction / np.linalg.norm(direction)
point_2 = push_distance * direction_normalized + point_1



def run_attempt(attempt_folder, point_1, point_2):
    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []
    p_utils.open_saved_scene(os.path.join(attempt_folder, "scene.csv"), attempt_folder, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)


    #push
    cylinderID = p_utils.create_cylinder(0.015 / 2, 0.05)
    p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))
    p_utils.print_image(view_matrix, proj_matrix, attempt_folder, extra_message="0_before")
    p_utils.push(point_2, cylinderID, dt, time_out=20.)
    p_utils.print_image(view_matrix, proj_matrix, attempt_folder, extra_message="1_after")

    # print positions and orientations of objects
    # Position is that of object's origin according to its .obj file, rather than the origin of the pybullet object.
    # Do this by subtracting out the world coordinates of the current COM.
    sim_data = []
    for object_ID in mobile_object_IDs:
        position, orientation = p.getBasePositionAndOrientation(object_ID)

        current_COM = p.getDynamicsInfo(object_ID, -1)[3]
        current_COM_oriented = p_utils.rotate_vector(current_COM, orientation)
        position_of_model_origin = np.array(position) - current_COM_oriented

        sim_data.append([position_of_model_origin[0], position_of_model_origin[1], position_of_model_origin[2], orientation[0], orientation[1], orientation[2], orientation[3]])
    file_handling.write_csv_file(os.path.join(attempt_folder, "results.csv"), "x,y,z,orientation_x,orientation_y,orientation_z,orientation_w", sim_data)

    p.resetSimulation()
    return sim_data


view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)

ground_truth_folder = os.path.join(test_dir,"ground_truth")
os.mkdir(ground_truth_folder)
original_scene_loc = os.path.join("scenes","scene_boxes_with_different_COMs.csv")
number_of_objects = 4
ground_truth_scene_loc = os.path.join(ground_truth_folder, "scene.csv")
file_handling.copy_file(original_scene_loc, ground_truth_scene_loc)





#run the ground truth simulation
ground_truth_movement_data = run_attempt(ground_truth_folder, point_1, point_2)


#find the acceptable COM bounds
com_x_range, com_y_range, com_z_range = p_utils.get_COM_bounds("cracker_box")

#generate and run scenes with alternate COMs
alternate_com_movement_data = []
for i in np.arange(5):
    # make directory for one attempt with alternate COMs
    attempt_dir_path = os.path.join(test_dir,"try_" + str(i).zfill(4))
    os.mkdir(attempt_dir_path)

    #create scene file
    new_COM_list = []
    for i in np.arange(number_of_objects):
        new_COM_list.append(p_utils.generate_point(com_x_range, com_y_range, com_z_range))
    p_utils.save_scene_with_shifted_COMs(original_scene_loc, os.path.join(attempt_dir_path,"scene.csv"), new_COM_list)

    this_scene_movement_data = run_attempt(attempt_dir_path, point_1, point_2)
    alternate_com_movement_data.append(this_scene_movement_data)

print(ground_truth_movement_data)
for alternate_com_movement_data_one_scene in alternate_com_movement_data:
    print(alternate_com_movement_data_one_scene)

p.disconnect()

