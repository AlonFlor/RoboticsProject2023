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



#TODO: have code similar to this for each object type, that returns a dictionary with object types as keys and their relevant com ranges and test points as values.
#find the acceptable COM bounds, in object coordinates
com_x_range, com_y_range, com_z_range = p_utils.get_COM_bounds("cracker_box")

#test points
test_points_x_amount = 20
test_points_y_amount = 20
test_points_z_amount = 5
test_points_x_coords =np.linspace(com_x_range[0],com_x_range[1], test_points_x_amount+2)[1:-1]
test_points_y_coords =np.linspace(com_y_range[0],com_y_range[1], test_points_y_amount+2)[1:-1]
test_points_z_coords =np.linspace(com_z_range[0],com_z_range[1], test_points_z_amount+2)[1:-1]
test_points = []
for i in np.arange(test_points_x_amount):
    for j in np.arange(test_points_y_amount):
        for k in np.arange(test_points_z_amount):
            test_points.append(np.array([test_points_x_coords[i],test_points_y_coords[j],test_points_z_coords[k]]))
num_test_points_per_object = test_points_x_amount * test_points_y_amount * test_points_z_amount



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



def get_loss(data_sim, data_gt, test_points):
    '''Calculate the loss as the sum of distances between simulated and ground truth for three test points (x,y,z) for each object.'''
    loss = 0.
    for i in np.arange(number_of_objects):
        pos, orn = data_sim[i]
        pos_gt, orn_gt = data_gt[i]

        #TODO edit this part when allowing multiple object types
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
for iter_num in np.arange(number_of_iterations):
    print("\n\nRound",iter_num+1,"out of",number_of_iterations)

    # make directory for new attempt with alternate COMs
    attempt_dir_path = os.path.join(test_dir,"try_" + str(iter_num).zfill(4))
    os.mkdir(attempt_dir_path)

    # create scene file
    p_utils.save_scene_with_shifted_COMs(original_scene_loc, os.path.join(attempt_dir_path, "scene.csv"), new_COM_list)

    #run the scene
    this_scene_movement_data = run_attempt(attempt_dir_path, point_1, point_2)
    #print("this_scene_movement_data",this_scene_movement_data)

    #update the losses
    loss = get_loss(this_scene_movement_data, ground_truth_movement_data, test_points) #TODO make it read different test points for each object type
    average_loss = loss/(num_test_points_per_object * number_of_objects)
    average_losses.append(average_loss)
    print("Loss:",loss)
    print("Average Loss:",average_loss)

    #find new locations for the object COMs
    updated_COMs = []
    for object_index in np.arange(number_of_objects):

        #get the center of mass of this object
        current_object_COM = new_COM_list[object_index]

        #get position and orientation data
        start_position, start_orientation = starting_data[object_index]
        position, orientation = this_scene_movement_data[object_index]
        position_gt, orientation_gt = ground_truth_movement_data[object_index]

        # get motions of each test point in ground truth and current scene
        test_point_differences = []
        for test_point in test_points: #TODO make it read different test points for each object type
            test_point_start = p_utils.get_world_space_point(test_point, start_position, start_orientation)
            test_point_gt = p_utils.get_world_space_point(test_point, position_gt, orientation_gt)
            test_point_sim = p_utils.get_world_space_point(test_point, position, orientation)
            test_point_differences.append(np.linalg.norm(test_point_sim - test_point_start) - np.linalg.norm(test_point_gt - test_point_start))

        #choose movements to this object's COM based on motion differences
        print("test_point_differences",test_point_differences)
        COM_changes = np.array([0.,0.,0.])
        for i in np.arange(num_test_points_per_object):
            diff = current_object_COM - test_points[i]
            diff_dir =  diff / np.linalg.norm(diff)
            COM_changes -= diff_dir * test_point_differences[i]

        #divide COM changes by the number of test points
        COM_changes /= num_test_points_per_object

        #TODO make sure center of rotation is accurate by seeing if object coords of COR remains consistent when the transformation is done using before and after poses,
        # then check if direction of COM change given by all of the test points matches direction of COM change given by just the COM test point,
        # and see if magnitude of COM change given by the COM test point is not too large (since it does not get divided by num_test_points_per_object nor canceled out by other
        # test points), consider checking for one object only. Also, multiply all derivatives by chord value.
        '''#get COM point differences for COM changes directly from current COM
        current_object_COM_world_coords_before = p_utils.get_world_space_point(current_object_COM, start_position, start_orientation)
        current_object_COM_world_coords_after = p_utils.get_world_space_point(current_object_COM, position, orientation)
        current_object_COM_world_coords_gt = p_utils.get_world_space_point(current_object_COM, position_gt, orientation_gt)
        current_COM_point_differences = np.linalg.norm(current_object_COM_world_coords_after - current_object_COM_world_coords_before) \
                                        - np.linalg.norm(current_object_COM_world_coords_gt - current_object_COM_world_coords_before)

        #get the center of rotation, for COM changes directly from current COM
        test_points_for_COR = [test_points[0], test_points[-1]]
        COR_2D = p_utils.center_of_rotation_2D(test_points_for_COR, start_position, start_orientation, position, orientation)
        if COR_2D is not None:
            COR = np.array([COR_2D[0], COR_2D[1], current_object_COM_world_coords_after[2]])
            COR_object_coords = p_utils.get_object_space_point(COR, position, orientation)

            #add COM changes from current COM
            r_COR_COM = current_object_COM - COR_object_coords
            r_COR_COM_dir = r_COR_COM / np.linalg.norm(r_COR_COM)
            COM_changes -= r_COR_COM_dir * current_COM_point_differences'''

        #clamp COM changes. TODO: consider a more principled way of doing this
        COM_changes_magn = np.linalg.norm(COM_changes)
        COM_changes_magn_limit = 0.03
        if COM_changes_magn > COM_changes_magn_limit:
            COM_changes /= COM_changes_magn
            COM_changes *= COM_changes_magn_limit

        #damp COM changes
        #COM_changes *= 0.9 ** iter_num

        print("changes to COM:",COM_changes)

        #define new COM for this object
        new_COM = current_object_COM + COM_changes

        #clamp object's new COM to bounds
        #TODO get specific com ranges for this object's type
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

