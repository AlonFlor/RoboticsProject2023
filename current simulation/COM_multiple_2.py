import pybullet as p
import file_handling
import pybullet_utilities as p_utils
import os
import numpy as np
import draw_data
import time


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

point_1 = np.array([0.2,0.,0.]) + cylinder_height_offset
point_2 = np.array([-0.3,0.,0.]) + cylinder_height_offset
#point_1 = np.array([0.15,0.,0.]) + cylinder_height_offset
#point_2 = np.array([-0.3,0.,0.]) + cylinder_height_offset

direction = point_2 - point_1
direction_normalized = direction / np.linalg.norm(direction)
point_2 = push_distance * direction_normalized + point_1





num_test_points_per_object = 125 #want to change this? Adjust the number in the get_com_bounds_and_test_points_for_object_type function in pybullet_utilities.py

object_type_com_bounds_and_test_points = {}
object_type_com_bounds_and_test_points["cracker_box"] = p_utils.get_com_bounds_and_test_points_for_object_type("cracker_box", 0.7, 0.8, 0.7)
object_type_com_bounds_and_test_points["master_chef_can"] = p_utils.get_com_bounds_and_test_points_for_object_type("master_chef_can", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["adjustable_wrench"] = p_utils.get_com_bounds_and_test_points_for_object_type("adjustable_wrench", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["pudding_box"] = p_utils.get_com_bounds_and_test_points_for_object_type("pudding_box", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["mustard_bottle"] = p_utils.get_com_bounds_and_test_points_for_object_type("mustard_bottle", 0.7, 0.7, 0.7)


#define scene, number of objects in that scene, and number of iterations
#original_scene_loc = os.path.join("scenes","four_items.csv")
#original_scene_loc = os.path.join("scenes","scene_boxes_with_different_COMs.csv")
original_scene_loc = os.path.join("scenes","scene_COM_overlay_one_object.csv")
number_of_objects = 1#4
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



def run_attempt(attempt_folder, point_1, point_2, get_starting_data=False):
    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []
    p_utils.open_saved_scene(os.path.join(attempt_folder, "scene.csv"), attempt_folder, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

    if get_starting_data:
        #get data before push
        starting_data = get_objects_positions_and_orientations(mobile_object_IDs)

    #push
    cylinderID = p_utils.create_cylinder(0.015 / 2, 0.05)
    p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))
    p_utils.print_image(view_matrix, proj_matrix, attempt_folder, extra_message="0_before")
    p_utils.push(point_2, cylinderID, dt, time_out=2.)
    p_utils.print_image(view_matrix, proj_matrix, attempt_folder, extra_message="1_after")

    #get data after push and reset simulation
    sim_data = get_objects_positions_and_orientations(mobile_object_IDs)
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)

    # print positions and orientations of objects
    sim_data_to_print = []
    for pos,orn in sim_data:
        sim_data_to_print.append([pos[0],pos[1],pos[2], orn[0],orn[1],orn[2],orn[3]])
    file_handling.write_csv_file(os.path.join(attempt_folder, "results.csv"), "x,y,z,orientation_x,orientation_y,orientation_z,orientation_w", sim_data)

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
starting_data, ground_truth_movement_data = run_attempt(ground_truth_folder, point_1, point_2, True)
#print("ground_truth_movement_data",ground_truth_movement_data)

#get the object coordinates version of the world z-axis for each object
object_angle_axes = []
for i in np.arange(number_of_objects):
    rotated_z_vector = p_utils.rotate_vector(np.array([0.,0.,1.]),p_utils.quat_inverse(starting_data[i][1]))
    direction_index = np.argmax(np.abs(rotated_z_vector))
    axis_sign = np.sign(rotated_z_vector[direction_index])
    print("axis_sign",axis_sign)
    object_angle_axes.append((direction_index, axis_sign))


#generate random COMs
current_COMs_list = []
for i in np.arange(number_of_objects):
    com_x_range,com_y_range,com_z_range = object_type_com_bounds_and_test_points[object_types[i]]["com_bounds"]
    current_COMs_list.append(p_utils.generate_point(com_x_range, com_y_range, com_z_range))

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
    this_scene_movement_data = run_attempt(attempt_dir_path, point_1, point_2)
    #print("this_scene_movement_data",this_scene_movement_data)


    #update angle errors
    average_angle_error = 0.
    sim_angles = []
    gt_angles = []
    for object_index in np.arange(number_of_objects):
        #get position and orientation data
        start_position, start_orientation = starting_data[object_index]
        position, orientation = this_scene_movement_data[object_index]
        position_gt, orientation_gt = ground_truth_movement_data[object_index]


        #get axis in object coords around which object rotates
        rotation_axis_index, rotation_axis_sign = object_angle_axes[object_index]

        #get angles
        sim_minus_start = p_utils.quaternion_difference(orientation, start_orientation)
        gt_minus_start = p_utils.quaternion_difference(orientation_gt, start_orientation)
        sim_axis, sim_angle = p_utils.quaternion_to_axis_angle(sim_minus_start)
        gt_axis, gt_angle = p_utils.quaternion_to_axis_angle(gt_minus_start)
        sim_angle = rotation_axis_sign*sim_axis[2]*sim_angle
        gt_angle = rotation_axis_sign*gt_axis[2]*gt_angle

        sim_angles.append(sim_angle)
        gt_angles.append(gt_angle)
        average_angle_error += (sim_angle - gt_angle)*180./np.pi
    average_angle_error /= number_of_objects
    average_angle_errors.append(average_angle_error)

    #update COM errors
    print("Current COM errors, COMs, gt COMs:")
    average_error = 0.
    for object_index in np.arange(number_of_objects):
        error = np.linalg.norm(ground_truth_COMs[object_index] - current_COMs_list[object_index])
        print(error,"\t\t", current_COMs_list[object_index], "\t\t", ground_truth_COMs[object_index])
        average_error += error
    average_error /= number_of_objects
    average_errors.append(average_error)

    #update the losses
    loss = 0.
    for object_index in np.arange(number_of_objects):
        test_points = object_type_com_bounds_and_test_points[object_types[object_index]]["test_points"]
        loss+= get_loss_for_object(this_scene_movement_data[object_index], ground_truth_movement_data[object_index], test_points)
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

        #get position and orientation data
        start_position, start_orientation = starting_data[object_index]
        position, orientation = this_scene_movement_data[object_index]
        position_gt, orientation_gt = ground_truth_movement_data[object_index]

        '''#draw image
        image_sim = np.zeros((100,100))
        image_gt = np.zeros((100,100))
        image = np.zeros((100,100))
        image_1 = np.zeros((100,100))
        image_2 = np.zeros((100,100))
        image_3 = np.zeros((100,100))
        com_x_range, com_y_range, com_z_range = object_type_com_bounds_and_test_points[object_types[object_index]]["com_bounds"]
        img_scale = 0.02
        for i in np.arange(100):
            for j in np.arange(100):
                test_point = np.array([-0.01, img_scale*(0.5*i-25.), img_scale*(0.5*j-25.)])
                test_point_start = p_utils.get_world_space_point(test_point, start_position, start_orientation)
                test_point_gt = p_utils.get_world_space_point(test_point, position_gt, orientation_gt)
                test_point_sim = p_utils.get_world_space_point(test_point, position, orientation)
                test_point_motion_sim = np.linalg.norm(test_point_sim - test_point_start)
                test_point_motion_gt = np.linalg.norm(test_point_gt - test_point_start)
                image_sim[i][j] = test_point_motion_sim
                image_gt[i][j] = - test_point_motion_gt
                image[i][j] = test_point_motion_sim - test_point_motion_gt

                if test_point[1] > com_y_range[0] and test_point[1] < com_y_range[1]:
                    if abs(test_point[2] - com_z_range[0]) < 0.5*img_scale or abs(test_point[2] - com_z_range[1]) < 0.5*img_scale:
                        image_sim[i][j] = 0.
                        image_gt[i][j] = 0.
                        image[i][j] = 0.
                        #image_1[i][j] = 0.
                        #image_2[i][j] = 0.
                        #image_3[i][j] = 0.
                if test_point[2] > com_z_range[0] and test_point[2] < com_z_range[1]:
                    if abs(test_point[1] - com_y_range[0]) < 0.5*img_scale or abs(test_point[1] - com_y_range[1]) < 0.5*img_scale:
                        image_sim[i][j] = 0.
                        image_gt[i][j] = 0.
                        image[i][j] = 0.
                        #image_1[i][j] = 0.
                        #image_2[i][j] = 0.
                        #image_3[i][j] = 0.
        
        draw_data.plt.imshow(image, cmap='binary')
        draw_data.plt.colorbar()
        draw_data.plt.show()'''

        #get angles
        rotation_axis_index = object_angle_axes[object_index][0]
        sim_angle = sim_angles[object_index]
        gt_angle = gt_angles[object_index]


        #get center of rotation for sim
        cor, cor_val = p_utils.center_of_rotation_2(rotation_axis_index, current_object_COM[rotation_axis_index], start_position, start_orientation, position, orientation)
        print("sim:",cor, cor_val)
        #get center of rotation for gt
        cor_gt, cor_gt_val = p_utils.center_of_rotation_2(rotation_axis_index, current_object_COM[rotation_axis_index], start_position, start_orientation, position_gt, orientation_gt)
        print("gt:",cor_gt, cor_gt_val)

        cor_to_c = current_object_COM - cor
        cor_to_c = cor_to_c / np.linalg.norm(cor_to_c)

        cor_gt_to_c_star = ground_truth_COMs[object_index] - cor_gt
        cor_gt_to_c_star = cor_gt_to_c_star / np.linalg.norm(cor_gt_to_c_star)
        print("are the paths from c to cor parallel?",np.dot(cor_gt_to_c_star, cor_to_c))

        learning_rate = 0.03
        COM_changes = learning_rate*(sim_angle-gt_angle)*np.sign(sim_angle)*cor_to_c


        #assume motion is planar without flipping.
        #TODO plan regarding flipping:
        # if it occurs in an object, find the coordinates of its simulated COM, look for the one closest to the edge, and move that coordinate towards the center.


        '''# current_COM_point_difference
        current_object_COM_world_coords_start = p_utils.get_world_space_point(current_object_COM, start_position, start_orientation)
        current_object_COM_world_coords_sim = p_utils.get_world_space_point(current_object_COM, position, orientation)
        current_object_COM_world_coords_gt = p_utils.get_world_space_point(current_object_COM, position_gt, orientation_gt)
        current_object_COM_motion_sim = np.linalg.norm(current_object_COM_world_coords_sim - current_object_COM_world_coords_start)
        current_object_COM_motion_gt = np.linalg.norm(current_object_COM_world_coords_gt - current_object_COM_world_coords_start)

        # get motions of each test point in ground truth and current scene
        test_point_differences = []
        test_points = object_type_com_bounds_and_test_points[object_types[object_index]]["test_points"]
        for test_point in test_points:
            test_point_start = p_utils.get_world_space_point(test_point, start_position, start_orientation)
            test_point_gt = p_utils.get_world_space_point(test_point, position_gt, orientation_gt)
            test_point_sim = p_utils.get_world_space_point(test_point, position, orientation)
            test_point_motion_sim = np.linalg.norm(test_point_sim - test_point_start)
            test_point_motion_gt = np.linalg.norm(test_point_gt - test_point_start)
            test_point_differences.append(test_point_motion_sim - current_object_COM_motion_sim - (test_point_motion_gt - current_object_COM_motion_gt))

        #TODO consider saving previous simulations, so if two COM candidates are encountered with changes pointing towards each other, we can look at a weighted average COM immediately.

        #choose movements to this object's COM based on motion differences
        #print("test_point_differences",test_point_differences)
        COM_changes = np.array([0.,0.,0.])
        for i in np.arange(num_test_points_per_object):
            diff = current_object_COM - test_points[i]
            diff_dir =  np.sign(diff)#diff / np.linalg.norm(diff)
            COM_changes -= diff_dir * 10.*(0.9**iter_num)*test_point_differences[i]

        #divide COM changes by the number of test points
        COM_changes /= num_test_points_per_object'''

        #clamp COM changes. TODO: consider a more principled way of doing this
        COM_changes_magn = np.linalg.norm(COM_changes)
        COM_changes_magn_limit = 0.03
        if COM_changes_magn > COM_changes_magn_limit:
            COM_changes /= COM_changes_magn
            COM_changes *= COM_changes_magn_limit


        print("changes to COM:",COM_changes)
        path_to_go = ground_truth_COMs[object_index] - current_object_COM
        print("path_to_go",path_to_go,"")

        print("dot of COM changes and actual path",np.dot(COM_changes / np.linalg.norm(COM_changes), path_to_go / np.linalg.norm(path_to_go)))
        print("\tnow same thing, but ignoring the rotation axis",np.dot(COM_changes[1:] / np.linalg.norm(COM_changes[1:]), path_to_go[1:] / np.linalg.norm(path_to_go[1:])))

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
        #exit()

    current_COMs_list = updated_COMs


print('Time to run simulations:', (time.perf_counter_ns() - sim_start) / 1e9, 's')


draw_data.plt.rcParams['figure.figsize'] = [9, 7.5]
sorted_average_errors = sorted(average_errors)
gap = 0.1*(sorted_average_errors[-1] - sorted_average_errors[0])
draw_data.plt.ylim(bottom=0.-gap, top=sorted_average_errors[-1]+gap)
draw_data.plot_variables_plain(range(len(average_errors)), "Iterations", average_errors, "Average COM error", out_dir=test_dir, show=False)

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
imgs_dir = os.path.join(test_dir, "comparison_images")
os.mkdir(imgs_dir)
for i in np.arange(num_iterations):
    try_folder_name = "try_"+str(i).zfill(4)
    p_utils.combine_images(os.path.join(test_dir,"ground_truth","1_after.png"),
                           os.path.join(test_dir,try_folder_name,"1_after.png"),
                           os.path.join(imgs_dir,try_folder_name+".png"))

p_utils.make_video(test_dir, imgs_dir, "try_", 8)

p.disconnect()
