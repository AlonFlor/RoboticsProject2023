import random

import pybullet as p
import numpy as np
import pybullet_utilities as p_utils
import file_handling

import os

# make directory for simulation files
test_num = 1
while os.path.exists("test" + str(test_num)):
    test_num += 1
test_dir = "test" + str(test_num)
os.mkdir(test_dir)

# physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.
view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)

object_scale = 0.015


#define pushing data
push_distance = 0.3
cylinder_height_offset = np.array([0., 0., 0.03])

point_1 = np.array([0.3,0.,0.]) + cylinder_height_offset
point_2 = np.array([-0.3,0.,0.]) + cylinder_height_offset

direction = point_2 - point_1
direction_normalized = direction / np.linalg.norm(direction)
point_2 = push_distance * direction_normalized + point_1


def run_full_push(attempt_dir,mobile_object_IDs, starting_positions, starting_orientations, cylinderID):
    #set up motion script
    motion_script = {}
    for object_ID in mobile_object_IDs:
        motion_script[object_ID] = []

    #reset the objects to the starting position
    for i in np.arange(len(mobile_object_IDs)):
        object_ID = mobile_object_IDs[i]
        pos = starting_positions[i]
        orn = starting_orientations[i]
        p.resetBasePositionAndOrientation(object_ID,pos,orn)
        p.resetBaseVelocity(object_ID, (0.,0.,0.), (0.,0.,0.))

    # push
    p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))
    motion_script[cylinderID] = []
    p_utils.print_image(view_matrix, proj_matrix, attempt_dir, extra_message="0_before")
    p_utils.push(point_2, cylinderID, dt, mobile_object_IDs=mobile_object_IDs, fps=24., motion_script=motion_script, time_out=20.)
    p_utils.print_image(view_matrix, proj_matrix, attempt_dir, extra_message="1_after")

    '''# print positions and orientations of objects at the end of the push
    sim_data = []
    for object_ID in mobile_object_IDs:
        position, orientation = p.getBasePositionAndOrientation(object_ID)
        sim_data.append([position_of_model_origin[0], position_of_model_origin[1], position_of_model_origin[2], orientation[0], orientation[1], orientation[2], orientation[3]])
    file_handling.write_csv_file(os.path.join(attempt_dir, "results.csv"),"x,y,z,orientation_x,orientation_y,orientation_z,orientation_w", sim_data)

    return sim_data'''

    return motion_script



def get_block_position_and_orientation(object_ID, link_index):
    if link_index == -1:
        return p.getBasePositionAndOrientation(object_ID)
    else:
        return p.getLinkState(object_ID, link_index)[:2]



def run_algorithm(scene_file, attempt_folder, point_1, point_2):
    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []

    #read the scene
    p_utils.open_saved_block_scene(scene_file, test_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list, object_scale)
    cylinderID = p_utils.create_cylinder(0.015 / 2, 0.05)

    #make lists of block masses and frictions, and lists of objects' starting positions and orientations
    masses = []
    frictions =[]
    starting_positions=[]
    starting_orientations=[]
    for object_ID in mobile_object_IDs:
        masses_this_object = []
        frictions_this_object = []

        mass, friction = p.getDynamicsInfo(object_ID, -1)[:2]
        masses_this_object.append(mass)
        frictions_this_object.append(friction)
        for j in np.arange(p.getNumJoints(object_ID)):
            mass, friction = p.getDynamicsInfo(object_ID, j)[:2]
            masses_this_object.append(mass)
            frictions_this_object.append(friction)

        masses.append(masses_this_object)
        frictions.append(frictions_this_object)

        starting_position, starting_orientation = p.getBasePositionAndOrientation(object_ID)
        starting_positions.append(starting_position)
        starting_orientations.append(starting_orientation)

    #
    gt_push = os.path.join(test_dir, "ground_truth_push")
    os.mkdir(gt_push)
    gt_motion_script = run_full_push(gt_push, mobile_object_IDs, starting_positions, starting_orientations, cylinderID)

    #reset masses and frictions
    for i in np.arange(len(mobile_object_IDs)):
        object_ID = mobile_object_IDs[i]
        for j in np.arange(p.getNumJoints(object_ID)+1):
            frictions[i][j] = 0.3
            masses[i][j] = 1./p.getNumJoints(object_ID)

    #TODO: Mention to professors that the interpolation assumption is even more dubious for multiple objects,
    #   due to objects further away being motionless until touched. Since time of touch is unknown, those further-away objects would start moving at t=0.


    #Algorithm 1 from Changkyu's paper.
    #TODO: Code Algorithm 1 from Changkyu's paper. Requires changing masses and moments of inertia (DO NOT FORGET MOMENTS OF INERTIA) and running run_push_attempt each time.

    loss = 0.
    for i in np.len(gt_motion_script[cylinderID]):

        # reset the pusher
        pusher_position, pusher_orientation = gt_motion_script[cylinderID][i]
        p.resetBasePositionAndOrientation(cylinderID, pusher_position, pusher_orientation)
        pusher_displacement_from_destination = cylinderID - np.array(pusher_position)
        pusher_dist_from_destination = np.linalg.norm(pusher_displacement_from_destination)
        pusher_speed = .1
        new_pusher_velocity = pusher_speed * pusher_displacement_from_destination / pusher_dist_from_destination
        p.resetBaseVelocity(cylinderID, (new_pusher_velocity[0], new_pusher_velocity[1], new_pusher_velocity[2]), (0., 0., 0.))

        #push
        p.stepSimulation()

        for j in np.arange(len(mobile_object_IDs)):
            object_ID = mobile_object_IDs[j]

            #get states of blocks
            actual_positions_and_angles = []
            for block_index in np.arange(len(masses[j])):
                actual_position, actual_orientation = get_block_position_and_orientation(object_ID, block_index-1)
                actual_position = np.array(actual_position)
                actual_angle = p.getEulerFromQuaternion(actual_orientation)[2]
                actual_positions_and_angles.append((actual_position,actual_angle))

            #reset shape to ground truth and get states of blocks
            time_val,position_0,position_1,position_2,orientation_0,orientation_1,orientation_2,orientation_3 = gt_motion_script[object_ID][i]
            gt_position = (position_0,position_1,position_2)
            gt_orientation = (orientation_0,orientation_1,orientation_2,orientation_3)
            p.resetBasePositionAndOrientation(object_ID, gt_position, gt_orientation)
            gt_positions_and_angles = []
            for block_index in np.arange(len(masses[j])):
                gt_position, gt_orientation = get_block_position_and_orientation(object_ID, block_index-1)
                gt_position = np.array(gt_position)
                gt_angle = p.getEulerFromQuaternion(gt_orientation)[2]
                gt_positions_and_angles.append((gt_position,gt_angle))

            #update masses and frictions
            for block_index in np.arange(len(masses[j])):
                state_position_difference = gt_positions_and_angles[block_index][0] - actual_positions_and_angles[block_index][0]
                state_angle_difference = gt_positions_and_angles[block_index][1] - actual_positions_and_angles[block_index][1]

                #each of the state differences is like a separate derivative which gets added up to the others.
                # change in m = learning rate * (dm/dx_0 + dm/dx_1 + dm/dx_2 + dI/dtheta * dm/dI)
                masses[j][block_index] += learning_rate * state_position_difference[0]
                masses[j][block_index] += learning_rate * state_position_difference[1]
                masses[j][block_index] += learning_rate * state_position_difference[2]  # change in z coord should be 0
                masses[j][block_index] += learning_rate * state_angle_difference * 6. #multiply by 6 to account for the fact that we are adjusting mass, not z-axis moment of inertia

                #TODO:
                # Define learning rate
                # Eliminate frictions by assuming they are the same as ground truth frictions.
                # Test the thing.



            #x is for each block.





        #reset masses and frictions
        loss

run_algorithm(os.path.join("scenes","block_planar_scenes","scene_1.csv"), test_dir, point_1, point_2)