import pybullet as p
import file_handling
import pybullet_utilities as p_utils
import os
import numpy as np
import draw_data
import time

#p_utils.save_scene_with_shifted_COMs(os.path.join("scenes","scene_one_box_2_ed.csv"),os.path.join("scenes","scene_one_box_2_ed_2.csv"),[np.array([-0.01,-0.01,0.08])])
#exit()


physicsClient = p.connect(p.DIRECT)
#physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

#set up camera
view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)


# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)
os.mkdir(test_dir)


#define pushing data
push_distance = 0.25#0.15
cylinder_height_offset = np.array([0., 0., 0.03])


def get_pushing_points(point_1_basic, point_2_basic):
    point_1 = point_1_basic + cylinder_height_offset
    point_2 = point_2_basic + cylinder_height_offset

    direction = point_2 - point_1
    direction_normalized = direction / np.linalg.norm(direction)
    point_2 = push_distance * direction_normalized + point_1
    return point_1, point_2

pushing_scenarios = []
#pushing_scenarios.append(get_pushing_points(np.array([0.2,-0.05,0.]), np.array([-0.3,-0.05,0.])))
#pushing_scenarios.append(get_pushing_points(np.array([0.,0.15,0.]), np.array([0.,-0.3,0.])))

pushing_scenarios.append(get_pushing_points(np.array([0.2,0.,0.]), np.array([-0.2,0.,0.])))
pushing_scenarios.append(get_pushing_points(np.array([0.05,0.4,0.]), np.array([0.05,-0.3,0.])))
pushing_scenarios.append(get_pushing_points(np.array([-0.05,-0.35,0.]), np.array([-0.05,0.3,0.])))

#pushing_scenarios.append(get_pushing_points(np.array([0.05,0.2,0.]), np.array([0.05,-0.3,0.])))


#TODO maybe read test points off of a file????
# Scenarios: Each simulated scenario carries 1 test file with any number of objects, and x pusher point pairs. x = ???
# Each push provides one contribution to the COM changes for each object.


#TODO remove test points inside objects, unless I am keeping them for losses purposes. If I have just a positition loss and an angle loss, no need for test points.
num_test_points_per_object = 125 #want to change this? Adjust the number in the get_com_bounds_and_test_points_for_object_type function in pybullet_utilities.py

object_type_com_bounds_and_test_points = {}
object_type_com_bounds_and_test_points["cracker_box"] = p_utils.get_com_bounds_and_test_points_for_object_type("cracker_box", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["master_chef_can"] = p_utils.get_com_bounds_and_test_points_for_object_type("master_chef_can", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["adjustable_wrench"] = p_utils.get_com_bounds_and_test_points_for_object_type("adjustable_wrench", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["pudding_box"] = p_utils.get_com_bounds_and_test_points_for_object_type("pudding_box", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["mustard_bottle"] = p_utils.get_com_bounds_and_test_points_for_object_type("mustard_bottle", 0.7, 0.7, 0.7)

# assume motion is planar without flipping.
# TODO plan regarding flipping:
# if it occurs in an object, find the coordinates of its simulated COM, look for the one closest to the edge, and move that coordinate towards the center.


#define scene, number of objects in that scene, and number of iterations
#original_scene_loc = os.path.join("scenes","scene_COM_overlay_one_object.csv")
original_scene_loc = os.path.join("scenes","scene_multi_boxes.csv")
#original_scene_loc = os.path.join("scenes","scene_one_box.csv")
number_of_objects = 4
max_number_of_iterations = 50


#create ground truth folder and copy scene
ground_truth_folder = os.path.join(test_dir,"ground_truth")
os.mkdir(ground_truth_folder)
ground_truth_scene_loc = os.path.join(ground_truth_folder, "scene.csv")
file_handling.copy_file(original_scene_loc, ground_truth_scene_loc)



def get_loss_for_object(data_sim, data_gt, test_points):
    '''Calculate the loss as the sum of distances between simulated and ground truth for three test points (x,y,z) for the object.'''
    loss = 0.
    pos, orn = data_sim
    pos_gt, orn_gt = data_gt

    for test_point in test_points:
        test_point_world_coords = p_utils.get_world_space_point(test_point, pos, orn)
        test_point_gt_world_coords = p_utils.get_world_space_point(test_point, pos_gt, orn_gt)
        loss += np.linalg.norm(test_point_world_coords-test_point_gt_world_coords)

    return loss


def get_objects_positions_and_orientations(mobile_object_IDs):
    # Position is that of object's origin according to its .obj file, rather than the origin of the pybullet object.
    # Do this by subtracting out the world coordinates of the current COM.
    sim_data = []
    for object_ID in mobile_object_IDs:
        position, orientation = p.getBasePositionAndOrientation(object_ID)

        current_COM = p.getDynamicsInfo(object_ID, -1)[3]
        current_COM_oriented = p_utils.rotate_vector(current_COM, orientation)
        position_of_model_origin = np.array(position) - current_COM_oriented

        sim_data.append((position_of_model_origin, orientation))

    return sim_data



def run_attempt(scene_folder, pushing_scenario_index, point_1, point_2, get_starting_data=False):
    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []
    push_folder = os.path.join(scene_folder,f"push_{pushing_scenario_index}")
    os.mkdir(push_folder)
    p_utils.open_saved_scene(os.path.join(scene_folder, "scene.csv"), push_folder, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

    if get_starting_data:
        #get data before push
        starting_data = get_objects_positions_and_orientations(mobile_object_IDs)

    #push
    cylinderID = p_utils.create_cylinder(0.015 / 2, 0.05)
    p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))
    p_utils.print_image(view_matrix, proj_matrix, push_folder, extra_message="0_before")
    p_utils.push(point_2, cylinderID, dt, time_out=2.)
    #p_utils.push(point_2, cylinderID, dt, mobile_object_IDs = mobile_object_IDs, fps = 24, view_matrix = view_matrix, proj_matrix = proj_matrix,
    #             imgs_dir = push_folder, available_image_num = 0, motion_script = None, time_out=2.)
    p_utils.print_image(view_matrix, proj_matrix, push_folder, extra_message="1_after")

    #get data after push and reset simulation
    sim_data = get_objects_positions_and_orientations(mobile_object_IDs)
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)

    # print positions and orientations of objects
    sim_data_to_print = []
    for pos,orn in sim_data:
        sim_data_to_print.append([pos[0],pos[1],pos[2], orn[0],orn[1],orn[2],orn[3]])
    file_handling.write_csv_file(os.path.join(push_folder, "results.csv"), "x,y,z,orientation_x,orientation_y,orientation_z,orientation_w", sim_data)

    if get_starting_data:
        return starting_data, sim_data
    return sim_data


#get the ground truth COMs
ground_truth_COMs = []
original_scene_data = file_handling.read_csv_file(original_scene_loc, [str, float, float, float, float, float, float, float, float, float, float, int])
for object_data in original_scene_data:
    ground_truth_COMs.append(np.array(object_data[1:4]))
ground_truth_COMs = np.array(ground_truth_COMs)

object_types = []
for object_data in original_scene_data:
    object_types.append(object_data[0])

#make sure ground truth COMs are in the COM bounds
for i in np.arange(number_of_objects):
    gt_COM = ground_truth_COMs[i]
    com_x_range,com_y_range,com_z_range = object_type_com_bounds_and_test_points[object_types[i]]["com_bounds"]
    out_of_range = (gt_COM[0] < com_x_range[0]) or (gt_COM[0] > com_x_range[1]) or \
                   (gt_COM[1] < com_y_range[0]) or (gt_COM[1] > com_y_range[1]) or \
                   (gt_COM[2] < com_z_range[0]) or (gt_COM[2] > com_z_range[1])
    if out_of_range:
        print("ground truth COM outside of defined range")
        exit()


sim_start = time.perf_counter_ns()

#run the ground truth simulation
starting_data = []
ground_truth_movement_data = []
for i,point_pair in enumerate(pushing_scenarios):
    point_1, point_2 = point_pair
    starting_data_p, ground_truth_movement_data_p = run_attempt(ground_truth_folder, i, point_1, point_2, True)
    starting_data.append(starting_data_p)
    ground_truth_movement_data.append(ground_truth_movement_data_p)
#print("ground_truth_movement_data",ground_truth_movement_data)

#get the object coordinates version of the world z-axis for each object
object_angle_axes = []
for i in np.arange(number_of_objects):
    rotated_z_vector = p_utils.rotate_vector(np.array([0.,0.,1.]),p_utils.quat_inverse(starting_data[0][i][1]))
    direction_index = np.argmax(np.abs(rotated_z_vector))
    axis_sign = np.sign(rotated_z_vector[direction_index])
    object_angle_axes.append((direction_index, axis_sign))


#generate random COMs
current_COMs_list = []
for i in np.arange(number_of_objects):
    com_x_range,com_y_range,com_z_range = object_type_com_bounds_and_test_points[object_types[i]]["com_bounds"]
    generated_com = p_utils.generate_point(com_x_range, com_y_range, com_z_range)
    rotation_axis_index = object_angle_axes[i][0]
    generated_com[rotation_axis_index] = 0.5*(com_x_range[0] + com_x_range[1]) #the guess for COM along the rotation axis is in the middle of the range.

    current_COMs_list.append(generated_com)


average_errors = []
average_losses = []
average_angle_errors = []
avg_loss_threshold = 0.005


#generate and run scenes with alternate COMs
import draw_data
num_iterations = 0
for iter_num in np.arange(max_number_of_iterations):
    num_iterations +=1
    print("\n\nRound",iter_num+1,"out of maximum",max_number_of_iterations)

    # make directory for new attempt with alternate COMs
    attempt_dir_path = os.path.join(test_dir,"try_" + str(iter_num).zfill(4))
    os.mkdir(attempt_dir_path)

    # create scene file
    p_utils.save_scene_with_shifted_COMs(original_scene_loc, os.path.join(attempt_dir_path, "scene.csv"), current_COMs_list)

    #run the scene
    this_scene_movement_data = []
    for i,point_pair in enumerate(pushing_scenarios):
        point_1, point_2 = point_pair
        this_scene_movement_data.append(run_attempt(attempt_dir_path, i, point_1, point_2))


    #update angle errors
    average_angle_error = 0.
    sim_angles = []
    gt_angles = []
    for pushing_scenario_index in np.arange(len(pushing_scenarios)):
        sim_angles.append([])
        gt_angles.append([])
        for object_index in np.arange(number_of_objects):
            #get position and orientation data
            start_position, start_orientation = starting_data[pushing_scenario_index][object_index]
            position, orientation = this_scene_movement_data[pushing_scenario_index][object_index]
            position_gt, orientation_gt = ground_truth_movement_data[pushing_scenario_index][object_index]


            #get axis in object coords around which object rotates
            rotation_axis_index, rotation_axis_sign = object_angle_axes[object_index]

            #get angles
            sim_minus_start = p_utils.quaternion_difference(orientation, start_orientation)
            gt_minus_start = p_utils.quaternion_difference(orientation_gt, start_orientation)
            sim_axis, sim_angle = p_utils.quaternion_to_axis_angle(sim_minus_start)
            gt_axis, gt_angle = p_utils.quaternion_to_axis_angle(gt_minus_start)
            #print(f"base axis angle for push{pushing_scenario_index}:\t\tsim:{sim_axis},{sim_angle}\t\tgt:{gt_axis},{gt_angle}")
            sim_angle = rotation_axis_sign*sim_axis[2]*sim_angle
            gt_angle = rotation_axis_sign*gt_axis[2]*gt_angle
            sim_angle = p_utils.restricted_angle_range(sim_angle)
            gt_angle = p_utils.restricted_angle_range(gt_angle)


            sim_angles[-1].append(sim_angle)
            gt_angles[-1].append(gt_angle)
            average_angle_error += (sim_angle - gt_angle)*180./np.pi
    average_angle_error /= number_of_objects
    average_angle_errors.append(average_angle_error)

    #update COM errors
    print("Current COM planar errors, COMs, gt COMs:")
    average_error = 0.
    for object_index in np.arange(number_of_objects):
        rotation_axis_index = object_angle_axes[object_index][0]
        ground_truth_COM_planar = ground_truth_COMs[object_index] + np.array([0.,0.,0.])
        ground_truth_COM_planar[rotation_axis_index]=0.
        current_COM_planar = current_COMs_list[object_index] + np.array([0.,0.,0.])
        current_COM_planar[rotation_axis_index]=0.
        error = np.linalg.norm(ground_truth_COM_planar - current_COM_planar)
        print("\t",error,"\t\t", current_COMs_list[object_index], "\t\t", ground_truth_COMs[object_index])
        average_error += error
    average_error /= number_of_objects
    average_errors.append(average_error)

    #update the losses
    loss = 0.
    for object_index in np.arange(number_of_objects):
        test_points = object_type_com_bounds_and_test_points[object_types[object_index]]["test_points"]
        for pushing_scenario_index in np.arange(len(pushing_scenarios)):
            loss+= get_loss_for_object(this_scene_movement_data[pushing_scenario_index][object_index], ground_truth_movement_data[pushing_scenario_index][object_index], test_points)
    average_loss = loss/(num_test_points_per_object * number_of_objects)
    average_losses.append(average_loss)
    print("Loss:",loss)
    print("Average Loss:",average_loss)

    #stop if the loss is low enough
    if average_loss < avg_loss_threshold:
        break

    #find new locations for the object COMs
    updated_COMs = []
    for object_index in np.arange(number_of_objects):

        #get the current center of mass of this object
        current_object_COM = current_COMs_list[object_index]
        print("current COM:", current_object_COM)

        COM_changes = np.array([0.,0.,0.])
        for pushing_scenario_index in np.arange(len(pushing_scenarios)):
            #get position and orientation data
            start_position, start_orientation = starting_data[pushing_scenario_index][object_index]
            position, orientation = this_scene_movement_data[pushing_scenario_index][object_index]
            position_gt, orientation_gt = ground_truth_movement_data[pushing_scenario_index][object_index]

            #get angles
            rotation_axis_index, rotation_axis_sign = object_angle_axes[object_index]
            sim_angle = sim_angles[pushing_scenario_index][object_index]
            gt_angle = gt_angles[pushing_scenario_index][object_index]


            #get center of rotation for sim
            cor, cor_val = p_utils.planar_center_of_rotation(sim_angle, rotation_axis_sign, start_position, start_orientation, position, orientation)
            print("cor",cor,"\t\tcor_val",cor_val)
            #get center of rotation for gt
            cor_gt, cor_gt_val = p_utils.planar_center_of_rotation(gt_angle, rotation_axis_sign, start_position, start_orientation, position_gt, orientation_gt)
            print("cor_gt",cor_gt,"\t\tcor_gt_val",cor_gt_val)

            #get the u vector, see paper for its use
            cor_to_c = current_object_COM - cor
            cor_to_c[rotation_axis_index] = 0.
            cor_to_c = cor_to_c / np.linalg.norm(cor_to_c)
            u =  np.sign(sim_angle)*cor_to_c

            #get the u vector for the ground truth to see if it matches
            cor_gt_to_c_star = ground_truth_COMs[object_index] - cor_gt
            cor_gt_to_c_star[rotation_axis_index] = 0.
            cor_gt_to_c_star = cor_gt_to_c_star / np.linalg.norm(cor_gt_to_c_star)
            print("u parallel???",np.dot(cor_to_c, cor_gt_to_c_star))


            print("u", u, "sim_angle-gt_angle",sim_angle-gt_angle)

            #update COM changes
            learning_rate = 0.03
            #print("sim_angle,gt_angle",sim_angle,gt_angle)
            COM_changes += learning_rate*(sim_angle-gt_angle)*u
            print(learning_rate*(sim_angle-gt_angle)*u)

            #is_correct_push_dir

            '''diff_of_CORs = cor_gt - cor
            diff_of_CORs[rotation_axis_index] = 0.
            diff_of_CORs_perp_u = diff_of_CORs - np.dot(diff_of_CORs,u)*u
            COM_changes += learning_rate*diff_of_CORs_perp_u
            print(learning_rate*diff_of_CORs_perp_u)'''

        '''#clamp COM changes. TODO: consider a more principled way of doing this
        COM_changes_magn = np.linalg.norm(COM_changes)
        COM_changes_magn_limit = 0.03
        if COM_changes_magn > COM_changes_magn_limit:
            COM_changes /= COM_changes_magn
            COM_changes *= COM_changes_magn_limit'''


        print("\nchanges to COM:",COM_changes)
        path_to_go = ground_truth_COMs[object_index] - current_object_COM
        print("path_to_go",path_to_go,"")

        print("dot of COM changes and actual path",np.dot(COM_changes / np.linalg.norm(COM_changes), path_to_go / np.linalg.norm(path_to_go)))

        #define new COM for this object
        new_COM = current_object_COM + COM_changes

        #clamp object's new COM to bounds
        com_x_range, com_y_range, com_z_range = object_type_com_bounds_and_test_points[object_types[object_index]]["com_bounds"]
        if new_COM[0] < com_x_range[0]:
            new_COM[0] = com_x_range[0]
        if new_COM[0] > com_x_range[1]:
            new_COM[0] = com_x_range[1]
        if new_COM[1] < com_y_range[0]:
            new_COM[1] = com_y_range[0]
        if new_COM[1] > com_y_range[1]:
            new_COM[1] = com_y_range[1]
        if new_COM[2] < com_z_range[0]:
            new_COM[2] = com_z_range[0]
        if new_COM[2] > com_z_range[1]:
            new_COM[2] = com_z_range[1]

        updated_COMs.append(new_COM)
        print()

    current_COMs_list = updated_COMs


print('Time to run simulations:', (time.perf_counter_ns() - sim_start) / 1e9, 's')


draw_data.plt.rcParams['figure.figsize'] = [9, 7.5]
sorted_average_errors = sorted(average_errors)
gap = 0.1*(sorted_average_errors[-1] - sorted_average_errors[0])
draw_data.plt.ylim(bottom=0.-gap, top=sorted_average_errors[-1]+gap)
draw_data.plot_variables_plain(range(len(average_errors)), "Iterations", average_errors, "Average COM planar error", out_dir=test_dir, show=False)

draw_data.plt.ylim(bottom=0.)
sorted_average_losses = sorted(average_losses)
gap = 0.1*(sorted_average_losses[-1] - sorted_average_losses[0])
draw_data.plt.ylim(bottom=0.-gap, top=sorted_average_losses[-1]+gap)
draw_data.plot_variables_plain(range(len(average_losses)), "Iterations", average_losses, "Average Loss", out_dir=test_dir, show=False)

sorted_average_angle_losses = sorted(average_angle_errors)
gap = 0.1*(sorted_average_angle_losses[-1] - sorted_average_angle_losses[0])
draw_data.plt.ylim(bottom=sorted_average_angle_losses[0]-gap, top=sorted_average_angle_losses[-1]+gap)
draw_data.plot_variables_plain(range(len(average_angle_errors)), "Iterations", average_angle_errors, "Average Angle Error (degrees)", out_dir=test_dir, show=False)

print(f"Took {num_iterations} iterations to converge to loss of {avg_loss_threshold}")

#show a comparison of the final images
for push_num in np.arange(len(pushing_scenarios)):
    imgs_dir = os.path.join(test_dir, f"push_{push_num}_comparison_images")
    os.mkdir(imgs_dir)
    for i in np.arange(num_iterations):
        try_folder_name = "try_"+str(i).zfill(4)
        p_utils.combine_images(os.path.join(test_dir,"ground_truth",f"push_{push_num}","1_after.png"),
                               os.path.join(test_dir,try_folder_name,f"push_{push_num}","1_after.png"),
                               os.path.join(imgs_dir,try_folder_name+".png"))

    p_utils.make_video(test_dir, imgs_dir, "try_", 8, f"push_{push_num}")

p.disconnect()
