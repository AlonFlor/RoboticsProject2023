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


def run_push_attempt(attempt_dir,mobile_object_IDs, starting_positions, starting_orientations):
    #reset the objects to the starting position
    for i in np.arange(len(mobile_object_IDs)):
        object_ID = mobile_object_IDs[i]
        pos = starting_positions[i]
        orn = starting_orientations[i]
        p.resetBasePositionAndOrientation(object_ID,pos,orn)
        p.resetBaseVelocity(object_ID, (0.,0.,0.), (0.,0.,0.))

    # push
    cylinderID = p_utils.create_cylinder(0.015 / 2, 0.05)
    p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))
    p_utils.print_image(view_matrix, proj_matrix, attempt_dir, extra_message="0_before")
    p_utils.push(point_2, cylinderID, dt, time_out=20.)
    p_utils.print_image(view_matrix, proj_matrix, attempt_dir, extra_message="1_after")

    # print positions and orientations of objects
    # Position is that of object's origin according to its .obj file, rather than the origin of the pybullet object.
    # Do this by subtracting out the world coordinates of the current COM.
    sim_data = []
    for object_ID in mobile_object_IDs:
        position, orientation = p.getBasePositionAndOrientation(object_ID)

        current_COM = p.getDynamicsInfo(object_ID, -1)[3]
        current_COM_oriented = p_utils.rotate_vector(current_COM, orientation)
        position_of_model_origin = np.array(position) - current_COM_oriented

        sim_data.append(
            [position_of_model_origin[0], position_of_model_origin[1], position_of_model_origin[2], orientation[0],
             orientation[1], orientation[2], orientation[3]])
    file_handling.write_csv_file(os.path.join(attempt_dir, "results.csv"),"x,y,z,orientation_x,orientation_y,orientation_z,orientation_w", sim_data)

    return sim_data

def run_algorithm(scene_file, attempt_folder, point_1, point_2):
    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []

    #read the scene
    p_utils.open_saved_block_scene(scene_file, test_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list, object_scale)

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

    run_push_attempt(os.path.join(test_dir,"ground_truth_push"), mobile_object_IDs, starting_positions, starting_orientations)

    #TODO: bring back the motion script. Not going to interpolate. Mention to professors that the interpolation assumption is even more dubious for multiple objects.
    #TODO: Code Algorithm 1 from Changkyu's paper. Requires changing masses and moments of inertia (DO NOT FORGET MOMENTS OF INERTIA) and running run_push_attempt each time.

run_algorithm(os.path.join("scenes","block_planar_scenes","scene_1.csv"), test_dir, point_1, point_2)