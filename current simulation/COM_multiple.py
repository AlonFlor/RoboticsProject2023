import pybullet as p
import file_handling
import pybullet_utilities as p_utils
import os
import numpy as np
import time


physicsClient = p.connect(p.DIRECT)
#physicsClient = p.connect(p.GUI)
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

#get the ground truth COMs
ground_truth_COMs = []
original_scene_data = file_handling.read_csv_file(original_scene_loc, [str, float, float, float, float, float, float, float, float, float, float, int])
for object_data in original_scene_data:
    ground_truth_COMs.append(np.array(object_data[1:4]))
ground_truth_COMs = np.array(ground_truth_COMs)


sim_start = time.perf_counter_ns()
number_of_simulations = 50

#run the ground truth simulation
ground_truth_movement_data = run_attempt(ground_truth_folder, point_1, point_2)


#find the acceptable COM bounds
com_x_range, com_y_range, com_z_range = p_utils.get_COM_bounds("cracker_box")

#generate and run scenes with alternate COMs
scene_paths = []
scene_COMs = []
scene_errors = []
for iter in np.arange(number_of_simulations):
    # make directory for one attempt with alternate COMs
    attempt_dir_path = os.path.join(test_dir,"try_" + str(iter).zfill(4))
    os.mkdir(attempt_dir_path)
    scene_paths.append(attempt_dir_path)

    #generate random centers of mass and create scene file
    new_COM_list = []
    for i in np.arange(number_of_objects):
        new_COM_list.append(p_utils.generate_point(com_x_range, com_y_range, com_z_range))
    scene_COMs.append(new_COM_list)
    p_utils.save_scene_with_shifted_COMs(original_scene_loc, os.path.join(attempt_dir_path,"scene.csv"), new_COM_list)

    #run the scene
    this_scene_movement_data = run_attempt(attempt_dir_path, point_1, point_2)

    #calculate error of generated scene vs ground truth
    spatial_diffs = 0.
    angular_diffs = 0.
    for i in np.arange(number_of_objects):
        position_gt = np.array(ground_truth_movement_data[i][:3])
        position_scene = np.array(this_scene_movement_data[i][:3])
        spatial_diffs += np.linalg.norm(position_gt - position_scene)

        orientation_gt = ground_truth_movement_data[i][-4:]
        orientation_scene = this_scene_movement_data[i][-4:]
        q_diff = p_utils.quaternion_difference(orientation_gt,orientation_scene)
        q_diff /= np.linalg.norm(q_diff)
        ang_magn = p_utils.quaternion_angular_magitude(q_diff)
        if ang_magn < 0:
            print(ang_magn)
            exit()
        angular_diffs += min(ang_magn, 2 * np.pi - ang_magn)

    scene_error = 10*spatial_diffs + angular_diffs      #spatial diffs are order of ~0.1 m , angular diffs are order of ~1 rad. Want roughly equal numerical contribution.
    scene_errors.append(scene_error)

print('Time to run simulations:', (time.perf_counter_ns() - sim_start) / 1e9, 's')

#Get probability score for each scene. Pre-normalization probability score = (total error) - (scene error)
scene_probability_scores = []
total_error_all_scenes = 0.
for error_score in scene_errors:
    total_error_all_scenes += error_score
for error_score in scene_errors:
    scene_probability_scores.append((total_error_all_scenes - error_score))
#normalize probability scores
prob_magn = 0.
for probability_score in scene_probability_scores:
    prob_magn += probability_score
for i in np.arange(number_of_simulations):
    scene_probability_scores[i] /= prob_magn



#get estimated COM by averaging the attempted COMs using their probability scores
COM_estimates = np.array([np.array([0., 0., 0.])] * number_of_objects)
for i in np.arange(number_of_simulations):
    scene_probability = scene_probability_scores[i]
    scene_COM_list = scene_COMs[i]
    for j in np.arange(number_of_objects):
        object_COM_estimate = np.array(scene_COM_list[j])
        COM_estimates[j] += scene_probability*object_COM_estimate

print(np.abs((ground_truth_COMs - COM_estimates) / ground_truth_COMs))

#TODO: maybe consider finding a way to refine the search for the correct COMs using the incorrect COMs.
#   This might end up turning this into a differential physics thing, where we differentiate the error or probability by the COM location.
#   Doing so would require moving from PyBullet to a differentiable simulator. Maybe consider doing that anyway?

p.disconnect()

