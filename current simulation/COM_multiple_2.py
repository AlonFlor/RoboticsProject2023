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
push_distance = 0.15
cylinder_height_offset = np.array([0., 0., 0.03])

point_1 = np.array([0.3,0.,0.]) + cylinder_height_offset
point_2 = np.array([-0.3,0.,0.]) + cylinder_height_offset

direction = point_2 - point_1
direction_normalized = direction / np.linalg.norm(direction)
point_2 = push_distance * direction_normalized + point_1



#find the acceptable COM bounds, in object coordinates
com_x_range, com_y_range, com_z_range = p_utils.get_COM_bounds("cracker_box")

#COM movement amount when COM gets updated
COM_movement_amount = 0.01



#define scene, number of objects in that scene, and number of iterations
original_scene_loc = os.path.join("scenes","scene_boxes_with_different_COMs.csv")
#original_scene_loc = os.path.join("scenes","scene_COM_overlay_one_object.csv")
number_of_objects = 4
number_of_iterations = 50


#create ground truth folder and copy scene
ground_truth_folder = os.path.join(test_dir,"ground_truth")
os.mkdir(ground_truth_folder)
ground_truth_scene_loc = os.path.join(ground_truth_folder, "scene.csv")
file_handling.copy_file(original_scene_loc, ground_truth_scene_loc)



def get_loss(data_sim, data_gt):
    '''Calculate the loss as the sum of distances between simulated and ground truth for three test points (x,y,z) for each object.'''
    loss = 0.
    for i in np.arange(number_of_objects):
        pos, orn = data_sim[i]
        pos_gt, orn_gt = data_gt[i]

        test_point_1 = p_utils.get_world_space_point(np.array([0.01,0.,0.]), pos, orn)
        test_point_1_gt = p_utils.get_world_space_point(np.array([0.01,0.,0.]), pos_gt, orn_gt)
        loss += np.linalg.norm(test_point_1-test_point_1_gt)
        test_point_2 = p_utils.get_world_space_point(np.array([0.,0.01,0.]), pos, orn)
        test_point_2_gt = p_utils.get_world_space_point(np.array([0.,0.01,0.]), pos_gt, orn_gt)
        loss += np.linalg.norm(test_point_2-test_point_2_gt)
        test_point_3 = p_utils.get_world_space_point(np.array([0.,0.,0.01]), pos, orn)
        test_point_3_gt = p_utils.get_world_space_point(np.array([0.,0.,0.01]), pos_gt, orn_gt)
        loss += np.linalg.norm(test_point_3-test_point_3_gt)

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


sim_start = time.perf_counter_ns()

#run the ground truth simulation
starting_data, ground_truth_movement_data = run_attempt(ground_truth_folder, point_1, point_2, True)
#print("ground_truth_movement_data",ground_truth_movement_data)


#generate random COMs
new_COM_list = []
for i in np.arange(number_of_objects):
    new_COM_list.append(p_utils.generate_point(com_x_range, com_y_range, com_z_range))

average_errors = []
average_losses = []

#print errors
print("Randomly generated COMs errors, COMs, gt_COMs:")
average_error = 0.
for i in np.arange(number_of_objects):
    error = np.linalg.norm(ground_truth_COMs[i] - new_COM_list[i])
    print(error,"\t\t",new_COM_list[i],"\t\t",ground_truth_COMs[i])
    average_error += error
average_error /= number_of_objects
average_errors.append(average_error)
print()


#generate and run scenes with alternate COMs
for iter in np.arange(number_of_iterations):
    print("\n\n")

    # make directory for new attempt with alternate COMs
    attempt_dir_path = os.path.join(test_dir,"try_" + str(iter).zfill(4))
    os.mkdir(attempt_dir_path)

    # create scene file
    p_utils.save_scene_with_shifted_COMs(original_scene_loc, os.path.join(attempt_dir_path, "scene.csv"), new_COM_list)

    #run the scene
    this_scene_movement_data = run_attempt(attempt_dir_path, point_1, point_2)
    #print("this_scene_movement_data",this_scene_movement_data)

    #update the losses
    loss = get_loss(this_scene_movement_data, ground_truth_movement_data)
    average_loss = loss/(3.*number_of_objects)
    average_losses.append(average_loss)
    print("\nLoss:",loss)
    print("Average Loss:",average_loss)

    #find new locations for the object COMs
    updated_COMs = []
    for object_index in np.arange(number_of_objects):

        #generate test points around each object's current COM
        test_points_this_object = []
        current_object_COM = new_COM_list[object_index]

        '''#each test point is average of current COM and edge of COM range.
        test_points_this_object.append(np.array([0.5*(current_object_COM[0] + com_x_range[0]),current_object_COM[1],current_object_COM[2]]))
        test_points_this_object.append(np.array([0.5*(current_object_COM[0] + com_x_range[1]),current_object_COM[1],current_object_COM[2]]))
        test_points_this_object.append(np.array([current_object_COM[0],0.5*(current_object_COM[1] + com_y_range[0]),current_object_COM[2]]))
        test_points_this_object.append(np.array([current_object_COM[0],0.5*(current_object_COM[1] + com_y_range[1]),current_object_COM[2]]))
        test_points_this_object.append(np.array([current_object_COM[0],current_object_COM[1],0.5*(current_object_COM[2] + com_z_range[0])]))
        test_points_this_object.append(np.array([current_object_COM[0],current_object_COM[1],0.5*(current_object_COM[2] + com_z_range[1])]))'''

        #each test point is at the edge of the range
        center_x_range = 0.5 * (com_x_range[0] + com_x_range[1])
        center_y_range = 0.5 * (com_y_range[0] + com_y_range[1])
        center_z_range = 0.5 * (com_z_range[0] + com_z_range[1])
        test_points_this_object.append(np.array([com_x_range[0], center_y_range, center_z_range]))
        test_points_this_object.append(np.array([com_x_range[1], center_y_range, center_z_range]))
        test_points_this_object.append(np.array([center_x_range, com_y_range[0], center_z_range]))
        test_points_this_object.append(np.array([center_x_range, com_y_range[1], center_z_range]))
        test_points_this_object.append(np.array([center_x_range, center_y_range, com_z_range[0]]))
        test_points_this_object.append(np.array([center_x_range, center_y_range, com_z_range[1]]))


        test_point_ratios = []
        # get motions of each test point in ground truth and current scene
        for test_point in test_points_this_object:
            test_point_start = p_utils.get_world_space_point(test_point, starting_data[object_index][0], starting_data[object_index][1])
            test_point_gt = p_utils.get_world_space_point(test_point, ground_truth_movement_data[object_index][0], ground_truth_movement_data[object_index][1])
            test_point_sim = p_utils.get_world_space_point(test_point, this_scene_movement_data[object_index][0], this_scene_movement_data[object_index][1])
            test_point_ratios.append(np.linalg.norm(test_point_sim - test_point_start) / np.linalg.norm(test_point_gt - test_point_start))

        #choose movements to this object's COM based on motion ratios
        print("test_point_ratios",test_point_ratios)
        COM_changes = np.array([0.,0.,0.])
        COM_changes[0] -= np.log(test_point_ratios[0])*COM_movement_amount
        COM_changes[0] += np.log(test_point_ratios[1])*COM_movement_amount
        COM_changes[1] -= np.log(test_point_ratios[2])*COM_movement_amount
        COM_changes[1] += np.log(test_point_ratios[3])*COM_movement_amount
        COM_changes[2] -= np.log(test_point_ratios[4])*COM_movement_amount
        COM_changes[2] += np.log(test_point_ratios[5])*COM_movement_amount
        #COM_changes *= 5
        print("size of changes:",COM_changes)

        #define new COM for this object
        new_COM = current_object_COM + COM_changes

        #clamp object's new COM to bounds
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

    new_COM_list = updated_COMs

    print("New COMs errors, COMs, gt COMs:")
    average_error = 0.
    for i in np.arange(number_of_objects):
        error = np.linalg.norm(ground_truth_COMs[i] - updated_COMs[i])
        print(error,"\t\t",new_COM_list[i],"\t\t",ground_truth_COMs[i])
        average_error += error
    average_error /= number_of_objects
    average_errors.append(average_error)


print('Time to run simulations:', (time.perf_counter_ns() - sim_start) / 1e9, 's')


draw_data.plt.rcParams['figure.figsize'] = [9, 7.5]
sorted_average_errors = sorted(average_errors)
gap = 0.1*(sorted_average_errors[-1] - sorted_average_errors[0])
draw_data.plt.ylim(bottom=0.-gap, top=sorted_average_errors[-1]+gap)
draw_data.plot_variables_plain(range(number_of_iterations+1), "Iterations", average_errors, "Average COM error", out_dir=test_dir, show=True)

draw_data.plt.ylim(bottom=0.)
sorted_average_losses = sorted(average_losses)
gap = 0.1*(sorted_average_losses[-1] - sorted_average_losses[0])
draw_data.plt.ylim(bottom=0.-gap, top=sorted_average_losses[-1]+gap)
draw_data.plot_variables_plain(range(number_of_iterations), "Iterations", average_losses, "Average Loss", out_dir=test_dir, show=True)


#show a comparison of the final images
imgs_dir = os.path.join(test_dir, "comparison_images")
os.mkdir(imgs_dir)
for i in np.arange(number_of_iterations):
    try_folder_name = "try_"+str(i).zfill(4)
    p_utils.combine_images(os.path.join(test_dir,"ground_truth","1_after.png"),
                           os.path.join(test_dir,try_folder_name,"1_after.png"),
                           os.path.join(imgs_dir,try_folder_name+".png"))

p_utils.make_video(test_dir, imgs_dir, "try_")

p.disconnect()

