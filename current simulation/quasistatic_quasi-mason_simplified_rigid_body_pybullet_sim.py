import random

import pybullet as p
import numpy as np
import pybullet_utilities as p_utils
import draw_data
import make_URDF

import os
import time








def get_rotated_object_axis(axis):
    object_basic_r = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])).reshape((3,3))
    return np.matmul(object_basic_r, axis)


def get_extreme_links_based_on_object_axes():
    """NOTE: for this method to work, object must be in its completely initial position and orientation"""

    loc = np.array(p.getBasePositionAndOrientation(objectID)[0])
    min_main_axis = -1, np.dot(loc, object_original_main_axis)
    max_main_axis = -1, np.dot(loc, object_original_main_axis)
    min_second_axis = -1, np.dot(loc, object_original_second_axis)
    max_second_axis = -1, np.dot(loc, object_original_second_axis)

    for i in range(num_links):
        loc = np.array(p.getLinkState(objectID, i)[0])
        main_axis_measure = np.dot(loc, object_original_main_axis)
        second_axis_measure = np.dot(loc, object_original_second_axis)
        if main_axis_measure < min_main_axis[1]:
            min_main_axis = i, main_axis_measure
        if main_axis_measure > max_main_axis[1]:
            max_main_axis = i, main_axis_measure
        if second_axis_measure < min_second_axis[1]:
            min_second_axis = i, second_axis_measure
        if second_axis_measure > max_second_axis[1]:
            max_second_axis = i, second_axis_measure

    return (min_main_axis[0], max_main_axis[0], min_second_axis[0], max_second_axis[0]), (min_main_axis[1], max_main_axis[1], min_second_axis[1], max_second_axis[1])


def get_object_link_position(index):
    if index != -1:
        return np.array(p.getLinkState(objectID, index)[0])
    else:
        return np.array(p.getBasePositionAndOrientation(objectID)[0])





def find_optimal_new_pushing_points_and_split_candidate_list(candidate_region_link_indices):
    """Given a list of links indices that make up a region where the center of mass could be, finds its geometric center.
     Then, find candidate start and end points for pushing by dissecting that region through the center."""

    #get locations of links in the candidate region
    locations = {}
    for index in candidate_region_link_indices:
        location = get_object_link_position(index)
        locations[index] = location

    #find a line that splits the new candidate region in two
    center = np.array([0.,0.,0.])
    for index in candidate_region_link_indices:
        center += locations[index]
    center /= len(candidate_region_link_indices)

    #get rotated object axes for the present configuration
    object_main_axis = get_rotated_object_axis(object_original_main_axis)
    object_second_axis = get_rotated_object_axis(object_original_second_axis)

    #find the optimal push axis, depending on the shape of the candidate region
    loc = locations[candidate_region_link_indices[0]]
    candidate_region_link_indices_min_main_axis = np.dot(loc, object_main_axis)
    candidate_region_link_indices_max_main_axis = np.dot(loc, object_main_axis)
    candidate_region_link_indices_min_second_axis = np.dot(loc, object_second_axis)
    candidate_region_link_indices_max_second_axis = np.dot(loc, object_second_axis)
    for index in candidate_region_link_indices[1:]:
        loc = locations[index]
        loc_main_axis = np.dot(loc, object_main_axis)
        loc_second_axis = np.dot(loc, object_second_axis)
        if loc_main_axis < candidate_region_link_indices_min_main_axis:
            candidate_region_link_indices_min_main_axis = loc_main_axis
        elif loc_main_axis > candidate_region_link_indices_max_main_axis:
            candidate_region_link_indices_max_main_axis = loc_main_axis
        if loc_second_axis < candidate_region_link_indices_min_second_axis:
            candidate_region_link_indices_min_second_axis = loc_second_axis
        elif loc_second_axis > candidate_region_link_indices_max_second_axis:
            candidate_region_link_indices_max_second_axis = loc_second_axis
    candidate_region_main_axis_span = candidate_region_link_indices_max_main_axis - candidate_region_link_indices_min_main_axis
    candidate_region_second_axis_span = candidate_region_link_indices_max_second_axis - candidate_region_link_indices_min_second_axis
    if candidate_region_main_axis_span > candidate_region_second_axis_span:
        chosen_axis = 0
    else:
        chosen_axis = 1

    #find pusher start and end points
    if chosen_axis == 0:
        min_second_axis = get_object_link_position(min_second_axis_link_index)
        max_second_axis = get_object_link_position(max_second_axis_link_index)

        new_pusher_start = center + (np.dot(max_second_axis-center,object_second_axis)+0.05) * object_second_axis
        new_pusher_end = center + (np.dot(min_second_axis-center,object_second_axis)-0.05) * object_second_axis
    else:
        min_main_axis = get_object_link_position(min_main_axis_link_index)
        max_main_axis = get_object_link_position(max_main_axis_link_index)

        new_pusher_start = center + (np.dot(max_main_axis-center,object_main_axis)+0.05) * object_main_axis
        new_pusher_end = center + (np.dot(min_main_axis-center,object_main_axis)-0.05) * object_main_axis

    #p.addUserDebugPoints([tuple(new_pusher_start), tuple(new_pusher_end)], [(1.,0.,0.),(0.,1.,0.)], 2.)


    # split candidate_region_link_indices depending on the side of the pushing axis the link is in
    candidate_region_right_link_indices = []
    candidate_region_left_link_indices = []
    push_axis = new_pusher_end - new_pusher_start
    for index in candidate_region_link_indices:
        # make the determination which region to add the link to
        add_link_to_candidate_region_right = np.sign(np.cross(locations[index] - new_pusher_start, push_axis)[2])
        if add_link_to_candidate_region_right > 0:
            candidate_region_right_link_indices.append(index)
        else:
            candidate_region_left_link_indices.append(index)

    return candidate_region_right_link_indices, candidate_region_left_link_indices, new_pusher_start, new_pusher_end



def push_object(pusher_end):

    test_point_velocities = []
    angles = []
    angular_velocities = []
    pushing_forces = []

    velocity = np.array([0., 0., 0.])
    angular_velocity = np.array([0., 0., 0.])
    initial_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
    current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]

    made_contact = False
    broke_contact = False
    broke_contact_countdown_reset = 25
    broke_contact_countdown = broke_contact_countdown_reset + 0

    #push
    pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
    while np.linalg.norm(np.array(pusher_position) - pusher_end) > 0.01:
        p.stepSimulation()
        time.sleep(dt)

        current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
        angles.append(current_angle)

        velocity, angular_velocity = p.getBaseVelocity(objectID)
        velocity = np.linalg.norm(np.array(velocity))
        test_point_velocities.append(velocity)
        angular_velocity = np.array(angular_velocity)
        angular_velocities.append(angular_velocity[2])
        #print("velocity =",velocity,"angular velocity =",angular_velocity)

        contact_points = p.getContactPoints(objectID, pusherID)
        contact_force = np.array([0., 0., 0.])
        pushing_contact_point = np.array([0.,0.,0.])
        for contact_point in contact_points:
            pushing_contact_point += np.array(contact_point[5])
            contact_force += contact_point[9] * np.array(contact_point[7])
        if len(contact_points) > 0 and not broke_contact:
            if not made_contact:
                made_contact = True
                broke_contact_countdown = broke_contact_countdown_reset + 0
            pushing_contact_point /= len(contact_points)
        else:
            if made_contact:
                broke_contact_countdown -= 1
                if broke_contact_countdown == 0:
                    broke_contact = True
        pushing_forces.append(contact_force)

        #reset the pusher
        pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
        pusher_position = (pusher_position[0], pusher_position[1], pusher_end[2]) #keep the height constant
        p.resetBasePositionAndOrientation(pusherID, pusher_position, (0.,0.,0.,1.))
        pusher_displacement_from_destination = pusher_end - np.array(pusher_position)
        pusher_dist_from_destination = np.linalg.norm(pusher_displacement_from_destination)
        pusher_speed = .15
        new_pusher_velocity = pusher_speed * pusher_displacement_from_destination/pusher_dist_from_destination
        p.resetBaseVelocity(pusherID, (new_pusher_velocity[0], new_pusher_velocity[1], new_pusher_velocity[2]), (0., 0., 0.))
        p.applyExternalForce(pusherID, -1, (0., 0., 9.8), (0.,0.,0.), p.LINK_FRAME) #antigravity

    return angles, test_point_velocities, angular_velocities



def get_push_sample(pusher_start, pusher_end):#, test_dir, push_number):
    ##make directories for files for this sample
    #push_file_dir = os.path.join(test_dir, "push_"+str(push_number))
    #os.mkdir(push_file_dir)

    #get initial object angle
    old_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]

    #reset pusher pose
    p.resetBasePositionAndOrientation(pusherID, (pusher_start[0], pusher_start[1], pusher_start[2]), (0., 0., 0., 1.))

    #push the object and plot stuff
    angles, test_point_velocities, angular_velocities = push_object(pusher_end)
    #draw_data.plot_variable_vs_time(angles, "angle", push_file_dir)
    #draw_data.plot_variable_vs_time(test_point_velocities, "velocity of test point", push_file_dir)
    #draw_data.plot_variable_vs_time(angular_velocities, "angular velocity", push_file_dir)
    #draw_data.plot_variable_vs_time(pushing_forces, "pushing force", push_dir)

    new_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]

    angular_displacement = new_angle - old_angle
    if angular_displacement < -3*np.pi/4:
        angular_displacement += 2*np.pi
    elif angular_displacement > 3*np.pi/4:
        angular_displacement -= 2*np.pi

    return angular_displacement








actual_mass = actual_com = actual_cof = actual_I = None
objectID = None
pusherID = None
dt = 1./240.
object_scale = 0.015
startPos = (0., 0., 1.*object_scale)
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
num_links = None

object_original_main_axis = np.array([-1., 0., 0.])
object_original_second_axis = np.array([0., 1., 0.])


def run_full_test(object_name):
    # make directory for simulation files
    test_num = 1
    while os.path.exists("test" + str(test_num)):
        test_num += 1
    test_dir = "test" + str(test_num)
    os.mkdir(test_dir)

    #make the file being tested
    make_URDF.write_URDF_from_text_file(object_name, test_dir)

    #physicsClient = p.connect(p.DIRECT)
    physicsClient = p.connect(p.GUI)
    p.setGravity(0,0,-9.8)

    #load plane
    global planeID
    planeID = p.loadURDF(os.path.join("object models","plane","plane.urdf"), useFixedBase=True)
    p.changeVisualShape(planeID, -1, rgbaColor=(0., 0., 0.9, 1.))

    #load pusher
    global pusherID
    pusher_radius = 0.01
    pusher_height = 0.1
    pusher_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=pusher_radius, height=pusher_height)
    pusher_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=pusher_radius, length=pusher_height)
    pusherID = p.createMultiBody(1., pusher_shapeID, pusher_visual_shapeID, (startPos[0], startPos[1]+0.1, startPos[2]+0.01), (0.,0.,0.,1.))
    p.changeVisualShape(pusherID, -1, rgbaColor=(0.5, 0.5, 0.5, 1.))

    #load object
    global objectID
    global num_links
    objectID = p.loadURDF(test_dir + "\\" + object_name + ".urdf", globalScaling=object_scale)
    num_links = p.getNumJoints(objectID)  # excludes base

    print("object:", objectID)
    print("pusher", pusherID)
    print("plane:", planeID)

    #scale the object's mass
    mass_scale = 1. / (num_links+1)
    p.changeDynamics(objectID, -1, mass_scale * p.getDynamicsInfo(objectID, -1)[0])
    for linkID in np.arange(num_links):
        p.changeDynamics(objectID, linkID, mass_scale * p.getDynamicsInfo(objectID, linkID)[0])

    '''#print object friction
    print("object base friction:",p.getDynamicsInfo(objectID, -1)[1])
    for linkID in np.arange(p.getNumJoints(objectID)):
        print("object link "+str(linkID) + " friction:",p.getDynamicsInfo(objectID, linkID)[1])'''

    global min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index
    links, distances_on_axes = get_extreme_links_based_on_object_axes()
    min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index = links


    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)


    #get actual mass distribution moments
    global actual_mass
    global actual_com
    global actual_cof
    global actual_I
    actual_mass, actual_com, actual_cof, actual_I = p_utils.get_actual_mass_com_cof_and_moment_of_inertia(objectID, num_links, object_scale) #global values
    print("actual_mass, actual_com, actual_I:",actual_mass, actual_com, actual_I)
    print("actual cof:", actual_cof)

    #set up initial candidate region
    #arbitrarily remove some candidates
    candidate_region_link_indices = [-1]
    for linkID in np.arange(num_links-2):
        candidate_region_link_indices.append(linkID)

    # repaint rejected links
    for index in np.arange(num_links):
        if index not in candidate_region_link_indices:
            p.changeVisualShape(objectID, index, rgbaColor=(0., 0., 0., 1.))

    #set up results file
    results_file = open(os.path.join(test_dir, "results_file.txt"), "w")

    pusher_height_disp = startPos[2]+0.05
    push_count = 0

    #set up the camera and start logging the video
    p_utils.set_up_camera(p.getBasePositionAndOrientation(objectID)[0], 0.75)
    loggerID = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.path.join(test_dir, "video.mp4"))

    while len(candidate_region_link_indices) > 1:
        # reset the camera
        p_utils.set_up_camera(p.getBasePositionAndOrientation(objectID)[0], 0.75)

        push_count += 1

        #find the best start and end pushing points to dissect the current candidate region.
        #also make two candidate lists from the current list, as split by the axis formed by the start and end pushing points
        candidate_region_right_link_indices, candidate_region_left_link_indices, pusher_start, pusher_end = \
            find_optimal_new_pushing_points_and_split_candidate_list(candidate_region_link_indices)
        pusher_start[2] = pusher_height_disp
        pusher_end[2] = pusher_height_disp

        #push the object and get the angular displacement
        angular_displacement = get_push_sample(pusher_start, pusher_end)#, test_dir, push_count)

        #write to results file
        results_file.write("push"+str(push_count)+":\n\tangular displacement = " + str(angular_displacement) + "\n\n")

        #check which candidate region can contain the center of mass, based on the angular displacement.
        if angular_displacement <= 0.:
            # angular displacement is zero or negative, so region to the right of the pushing axis is the candidate region
            candidate_region_link_indices = candidate_region_right_link_indices
            rejected_links = candidate_region_left_link_indices
        else:
            # angular displacement is positive, so region to the left of the pushing axis is the candidate region
            candidate_region_link_indices = candidate_region_left_link_indices
            rejected_links = candidate_region_right_link_indices

        # repaint rejected links
        for index in rejected_links:
            p.changeVisualShape(objectID, index, rgbaColor=(0., 0., 0., 1.))

        #wait for a bit
        time_extra = 0.5
        while time_extra > 0:
            p.resetBaseVelocity(pusherID, (0., 0., 0.), (0., 0., 0.))
            p.applyExternalForce(pusherID, -1, (0., 0., 9.8), (0.,0.,0.), p.LINK_FRAME) #antigravity

            time_extra -= dt
            p.stepSimulation()
            time.sleep(dt)

    #stop logging the video
    p.stopStateLogging(loggerID)

    #write down results
    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)
    found_com = get_object_link_position(candidate_region_link_indices[0])
    com_error = np.linalg.norm(found_com[:2]-actual_com[:2])

    results_file.write("found COM:\t" + str(found_com[:2]) + "\n")
    results_file.write("actual COM:\t" + str(actual_com[:2]) + "\n")
    results_file.write("COM error:\t" + str(com_error) + "\n")
    results_file.close()



    p.disconnect()

run_full_test("spray_gun_uniform")
run_full_test("spray_gun")
run_full_test("snack_uniform")
run_full_test("snack")
run_full_test("book_uniform")
run_full_test("book")
run_full_test("wrench_uniform")
run_full_test("wrench")
run_full_test("hammer_uniform")
run_full_test("hammer")