import random

import pybullet as p
import numpy as np
import draw_data
import make_URDF
import file_handling
import scipy
import os

import sklearn.gaussian_process as gp
import sklearn.linear_model as lin_mod
import sklearn.model_selection as mod_sel

import time

#time.sleep(20)



def get_actual_mass_com_and_moment_of_inertia():
    #count = 1
    masses = []
    mass = p.getDynamicsInfo(objectID, -1)[0] #base mass
    masses.append(mass)
    #print(count-1,mass,p.getDynamicsInfo(objectID,-1)[1])

    loc_weighed_mass = masses[0]*np.array(list(p.getBasePositionAndOrientation(objectID)[0])) #base location

    for i in range(num_links):
        this_mass = p.getDynamicsInfo(objectID, i)[0]
        masses.append(this_mass)
        #print(count,this_mass,p.getDynamicsInfo(objectID,i)[1])

        this_loc = p.getLinkState(objectID, i)[0]
        mass += this_mass
        loc_weighed_mass += np.array(list(this_loc))*this_mass

        #count += 1
    com = loc_weighed_mass/mass

    object_scale_factor = object_scale**2
    I = object_scale_factor*masses[0]/6. + masses[0]*(np.linalg.norm(np.array(list(p.getBasePositionAndOrientation(objectID)[0])) - com)**2)
    for i in range(num_links):
        #print("I",I)
        I += object_scale_factor*masses[i+1]/6. + masses[i+1]*(np.linalg.norm(np.array(list(p.getLinkState(objectID, i)[0])) - com) ** 2)


    '''other_I = p.getDynamicsInfo(objectID,-1)[2][1] + masses[0]*(np.linalg.norm(np.array(list(p.getBasePositionAndOrientation(objectID)[0])) - com)**2)
    for i in range(num_links):
        print("other I", other_I,p.getDynamicsInfo(objectID,i)[2])
        other_I += p.getDynamicsInfo(objectID,i)[2][1] + masses[i+1]*(np.linalg.norm(np.array(list(p.getLinkState(objectID, i)[0])) - com) ** 2)
    print("\n\n\n")
    print("object_scale",object_scale)
    print("I",I)
    print("other I",other_I)
    exit()'''

    return mass, com, I




def transformation_matrix(R, pos):
    return np.block([
        [R, pos.reshape(3,1)],
        [np.array([[0., 0., 0., 1.]])]
    ])




def get_point_on_object_in_world_coordinates(original_point):
    position, orientation = p.getBasePositionAndOrientation(objectID)
    r = np.array(list(p.getMatrixFromQuaternion(orientation))).reshape((3,3))
    tr = transformation_matrix(r, np.array(list(position)))
    return np.matmul(tr, np.append(original_point, [1]))[:3]




def get_object_axes():
    rotation = np.array(list(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(objectID)[1]))).reshape((3, 3))
    return np.matmul(rotation,object_original_main_axis), np.matmul(rotation,object_original_second_axis)




def get_extreme_links_based_on_object_axes():
    #NOTE: for this method to work, object must be in its initial position and orientation

    loc = np.array(list(p.getBasePositionAndOrientation(objectID)[0]))
    min_main_axis = -1, np.dot(loc, object_original_main_axis)
    max_main_axis = -1, np.dot(loc, object_original_main_axis)
    min_second_axis = -1, np.dot(loc, object_original_second_axis)
    max_second_axis = -1, np.dot(loc, object_original_second_axis)

    for i in range(num_links):
        loc = np.array(list(p.getLinkState(objectID, i)[0]))
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

    return min_main_axis[0], max_main_axis[0], min_second_axis[0], max_second_axis[0]




def get_object_link_position(index):
    if index != -1:
        return np.array(list(p.getLinkState(objectID, index)[0]))
    else:
        return np.array(list(p.getBasePositionAndOrientation(objectID)[0]))




def five_point_central_difference(index, series):
    #index should be at least 2 and at most len(series)-3
    return (series[index-2] - 8*series[index-1] + 8*series[index+1] - series[index+2]) / (12.*dt)





def nearest_link_to_point(point):
    loc = np.array(list(p.getBasePositionAndOrientation(objectID)[0]))
    index = -1
    shortest_dist = np.linalg.norm(loc - point)

    for i in range(num_links):
        loc = np.array(list(p.getLinkState(objectID, i)[0]))
        dist = np.linalg.norm(loc - point)
        if dist < shortest_dist:
            index = i
            shortest_dist = dist
    return index, shortest_dist




def get_push_start_and_end_positions(axis_to_be_probed, motion_toward_robot):
    main_axis, second_axis = get_object_axes()

    min_main_axis_link_pos = get_object_link_position(min_main_axis_link_index)
    max_main_axis_link_pos = get_object_link_position(max_main_axis_link_index)
    min_second_axis_link_pos = get_object_link_position(min_second_axis_link_index)
    max_second_axis_link_pos = get_object_link_position(max_second_axis_link_index)

    min_main_axis_link_pos[2] = max_main_axis_link_pos[2] = min_second_axis_link_pos[2] = max_second_axis_link_pos[2] = 0.04

    param_in_axis_to_be_probed = random.random()
    #param_in_axis_to_be_probed_1 = random.random()
    #param_in_axis_to_be_probed_2 = random.random()
    distance_in_other_axis_1 = 0.05*random.random() + 0.05
    distance_in_other_axis_2 = 0.05*random.random() + 0.05
    point_1 = None
    point_2 = None

    if axis_to_be_probed == 0:
        distance = np.dot(main_axis, max_main_axis_link_pos - min_main_axis_link_pos)
        point_in_axis_to_be_probed = min_main_axis_link_pos + param_in_axis_to_be_probed*distance*main_axis
        #point_in_axis_to_be_probed_1 = min_main_axis_link_pos + param_in_axis_to_be_probed_1*distance*main_axis
        #point_in_axis_to_be_probed_2 = min_main_axis_link_pos + param_in_axis_to_be_probed_2*distance*main_axis
        displacement_in_other_axis_1 = np.dot(min_second_axis_link_pos - point_in_axis_to_be_probed, second_axis) - distance_in_other_axis_1
        displacement_in_other_axis_2 = np.dot(max_second_axis_link_pos - point_in_axis_to_be_probed, second_axis) + distance_in_other_axis_2
        point_1 = point_in_axis_to_be_probed + displacement_in_other_axis_1*second_axis
        point_2 = point_in_axis_to_be_probed + displacement_in_other_axis_2*second_axis
    else:
        distance = np.dot(second_axis, max_second_axis_link_pos - min_second_axis_link_pos)
        point_in_axis_to_be_probed = min_second_axis_link_pos + param_in_axis_to_be_probed*distance*second_axis
        #point_in_axis_to_be_probed_1 = min_second_axis_link_pos + param_in_axis_to_be_probed_1*distance*second_axis
        #point_in_axis_to_be_probed_2 = min_second_axis_link_pos + param_in_axis_to_be_probed_2*distance*second_axis
        displacement_in_other_axis_1 = np.dot(min_main_axis_link_pos - point_in_axis_to_be_probed, main_axis) - distance_in_other_axis_1
        displacement_in_other_axis_2 = np.dot(max_main_axis_link_pos - point_in_axis_to_be_probed, main_axis) + distance_in_other_axis_2
        point_1 = point_in_axis_to_be_probed + displacement_in_other_axis_1*main_axis
        point_2 = point_in_axis_to_be_probed + displacement_in_other_axis_2*main_axis

    #choose which point is start and which point is end
    start_point = None
    end_point = None
    if motion_toward_robot:
        if np.linalg.norm(point_1) > np.linalg.norm(point_2):
            start_point = point_1
            end_point = point_2
        else:
            start_point = point_2
            end_point = point_1
    else:
        if np.linalg.norm(point_1) > np.linalg.norm(point_2):
            start_point = point_2
            end_point = point_1
        else:
            start_point = point_1
            end_point = point_2

    return start_point, end_point




def abort_motion():
    # pull up
    pull_up_position = [0.4, 0., 0.3]
    joint_pos_list = p.calculateInverseKinematics(robotID, 19, pull_up_position)
    for i in np.arange(len(joint_pos_list)):
        p.setJointMotorControl2(robotID, i, controlMode=p.POSITION_CONTROL, targetPosition=joint_pos_list[i])
    time_amount = 0.
    while time_amount < 0.5:
        p.stepSimulation()
        time_amount += dt
        time.sleep(dt)




def set_robot_end_effector_pose(end_effector_position, end_effector_orientation=None):

    current_end_effector_position = np.array(list(p.getLinkState(robotID, 19, 1)[0]))
    target_position = np.array(end_effector_position)
    abort_motion_timer = 0
    while np.linalg.norm(current_end_effector_position - target_position) > 0.003:
        joint_pos_list = None
        if end_effector_orientation is None:
            joint_pos_list = p.calculateInverseKinematics(robotID, 19, end_effector_position)
        else:
            joint_pos_list = p.calculateInverseKinematics(robotID, 19, end_effector_position, end_effector_orientation)

        for i in np.arange(len(joint_pos_list)):
            #p.resetJointState(robotID, i, joint_pos_list[i])
            p.setJointMotorControl2(robotID, i, controlMode=p.POSITION_CONTROL, targetPosition=joint_pos_list[i], targetVelocity=0.)
        joint_motion = 1.
        joint_positions = []
        for i in np.arange(len(joint_pos_list)):
            joint_positions.append(p.getJointState(robotID, i)[0])

        # loop until the robot movement is basically 0
        while joint_motion > 0.001:
            p.stepSimulation()

            joint_motion = 0.
            for i in np.arange(len(joint_pos_list)):
                joint_motion += abs(p.getJointState(robotID, i)[0] - joint_positions[i])
                joint_positions[i] = p.getJointState(robotID, i)[0]

            '''#print status of the end effector
            current_end_effector_state = p.getLinkState(robotID, 19)
            print("current end effector loc:", current_end_effector_state[0])#, current_end_effector_state[1])
            print("\ttarget:",end_effector_position)
            if end_effector_orientation is not None:
                print("current end effector orientation:",current_end_effector_state[1])
                print("\ttarget:",end_effector_orientation)
            print("joint_motion",joint_motion)'''
            print("abort motion timer: ", str(abort_motion_timer)+"/"+str(abort_motion_threshold))
            #print("\n")

            # code to abort motion if robot gets stuck
            if abort_motion_timer >= abort_motion_threshold:
                print("Failed to get to start position. Aborting push.")
                abort_motion()
                return 1
            abort_motion_timer += dt

            time.sleep(dt)

        current_end_effector_position = np.array(list(p.getLinkState(robotID, 19, 1)[0]))

        # code to abort motion if robot gets stuck
        if abort_motion_timer >= abort_motion_threshold:
            print("Failed to get to start position. Aborting push.")
            abort_motion()
            return 1
    return 0




def push_object(start_position, end_position, axis_to_be_probed):
    # first, get robot to move to start position

    end_effector_orientation = np.array([0.22727178037166595, 0.676620602607727, -0.5784782767295837, 0.39483538269996643])
    #np.array([0.6123724, 0.3535534, -0.3535534, 0.6123724])
    # [ 0.6532815, 0.2705981, 0.2705981, 0.6532815 ]#[ 0.5, 0, 0.5, 0.7071067811865475244 ]#[0., 0, 0, 1.]#[0.7071067811865475244, 0, 0, 0.7071067811865475244]
    # [ 0.6532815, 0.2705981, -0.2705981, 0.6532815 ]
    end_effector_orientation = list(end_effector_orientation / np.linalg.norm(end_effector_orientation))
    reset_result = set_robot_end_effector_pose(start_position, end_effector_orientation)
    if reset_result == 1:
        return None #Failed to get to start position. Aborting push.
    time.sleep(0.1)
    print("starting push")

    # push the object and collect the data

    joint_pos_list = p.calculateInverseKinematics(robotID, 19, end_position, end_effector_orientation)
    current_end_effector_position = np.array(list(p.getLinkState(robotID, 19, 1)[0]))

    #information needed:
    #object velocities - at two separate points
    #object angular velocities
    #object contact points along axis - to find point along axis where contact happens.
    #object pushing forces
    #pushing angles
    #robot joint torques
    object_velocities_x = []
    object_velocities_y = []
    object_angular_velocities = []
    object_contact_points_along_axis = []
    object_pushing_forces = []
    pushing_angles = []
    robot_joint_torques = []
    object_contact_time_step_indices = []

    actual_pushing_forces = [] #actual pushing forces for troubleshooting purposes
    actual_friction_forces = [] #actual friction forces for troubleshooting purposes
    actual_pushing_torques = [] #actual pushing torques for troubleshooting purposes
    actual_friction_torques = [] #actual friction torques for troubleshooting purposes

    for i in np.arange(len(joint_pos_list)):
        #p.resetJointState(robotID, i, joint_pos_list[i])
        p.setJointMotorControl2(robotID, i, controlMode=p.POSITION_CONTROL, targetPosition=joint_pos_list[i], targetVelocity=.2, maxVelocity=.4)
    #loop until the object's contact with the robot has been established and is then lost, plus time_offset
    time_offset = 0.15
    bumped = False
    detached_contact = False
    continue_with_sim = True
    abort_motion_timer = 0.
    while continue_with_sim:
        p.stepSimulation()

        # record object velocity and angular velocity
        velocity = np.array(list(p.getBaseVelocity(objectID)[0]))
        angular_velocity = np.array(list(p.getBaseVelocity(objectID)[1]))
        object_velocities_x.append(velocity[0])
        object_velocities_y.append(velocity[1])
        object_angular_velocities.append(angular_velocity[2])

        # if the robot finger contacts the object, extract data related to the contact
        contact_points = p.getContactPoints(robotID, objectID)
        if len(contact_points) > 0 and (not detached_contact):
            bumped = True

            #get the axes
            axes = get_object_axes()
            axis_vector = axes[axis_to_be_probed]
            normal_vector = axes[1-axis_to_be_probed]
            #print("axis_vector",axis_vector)
            #print("normal_vector",normal_vector)

            # get the current actual COM for troubleshooting purposes
            current_com_world_coords = get_point_on_object_in_world_coordinates(actual_com_object_frame)
            print("current_com_world_coords",current_com_world_coords)

            #get the contact point
            contact_point = np.array([0., 0., 0.])
            force = 0.
            actual_pushing_force = np.array([0., 0., 0.])
            actual_pushing_torque = np.array([0., 0., 0.])
            for i in np.arange(len(contact_points)):
                contact_point += np.array(contact_points[i][6])
                force_vector = -1*np.array(list(contact_points[i][7]))
                force_direction_adjustment = np.dot(normal_vector, force_vector)
                #print("force_vector",force_vector)
                #print("force_direction_adjustment",force_direction_adjustment)
                #print("\t\t\tforces magns:", contact_points[i][9], contact_points[i][10], contact_points[i][12])

                #friction1_direction_adjustment = np.dot(normal_vector, np.array(list(contact_points[i][11])))
                #friction2_direction_adjustment = np.dot(normal_vector, np.array(list(contact_points[i][13])))

                force += contact_points[i][9] * force_direction_adjustment #+ \
                #         contact_points[i][10] * friction1_direction_adjustment + contact_points[i][12] * friction2_direction_adjustment

                # get the actual force and torque for troubleshooting purposes
                actual_moment = np.array(contact_points[i][6]) - current_com_world_coords
                actual_force = contact_points[i][9] * force_vector
                actual_pushing_force += actual_force
                actual_pushing_torque += np.cross(actual_moment, actual_force)

            #get actual friction forces and torques for troubleshooting purposes
            actual_friction_force = np.array([0., 0., 0.])
            actual_friction_torque = np.array([0., 0., 0.])
            friction_points = p.getContactPoints(planeID, objectID)
            for i in np.arange(len(friction_points)):
                friction_moment = np.array(friction_points[i][6]) - current_com_world_coords
                friction_vector_1 = np.array(list(friction_points[i][11]))
                friction_vector_2 = np.array(list(friction_points[i][13]))
                actual_friction = friction_points[i][10] * friction_vector_1 + friction_points[i][12] * friction_vector_2
                actual_friction_force += actual_friction
                actual_friction_torque += np.cross(friction_moment, actual_friction)
            actual_friction_forces.append(actual_friction_force)
            actual_friction_torques.append(actual_friction_torque[2])

            # get the actual pushing forces and torques, for troubleshooting purposes
            print("\t\t\t\tactual torque:",actual_pushing_torque)
            actual_pushing_forces.append(actual_pushing_force)
            actual_pushing_torques.append(actual_pushing_torque[2])

            #contact point recorded is average of detected contact points. They should be close to each other on the xy plane.
            contact_point /= len(contact_points)

            #get the contact point on the object's axis to be probed
            min_pos = None
            if axis_to_be_probed == 0:
                min_pos = get_object_link_position(min_main_axis_link_index)
            else:
                min_pos = get_object_link_position(min_second_axis_link_index)
            contact_point_along_axis = np.dot(contact_point - min_pos, axis_vector)
            object_contact_points_along_axis.append(contact_point_along_axis)

            # get the pushing angle
            robot_finger_velocity = np.array(list(p.getLinkState(objectID, 19, 1)[6]))
            robot_finger_velocity_normalized = robot_finger_velocity / np.linalg.norm(robot_finger_velocity)
            pushing_angles.append(np.arccos(np.dot(robot_finger_velocity_normalized, axis_vector)))

            # get the pushing force felt by the object
            object_pushing_forces.append(force)

            #get robot joint torques
            joint_torques_this_time_step = []
            for i in np.arange(len(joint_pos_list)):
                joint_torques_this_time_step.append(p.getJointState(robotID, i)[3])
            robot_joint_torques.append(joint_torques_this_time_step)

            #save the index of this time step, since it is a time step where contact occurred.
            object_contact_time_step_indices.append(len(object_velocities_x) - 1)
        elif bumped:
            detached_contact = True
        if bumped and detached_contact:
            time_offset -= dt
        if time_offset <= 0:
            continue_with_sim = False
        current_end_effector_position = np.array(list(p.getLinkState(robotID, 19, 1)[0]))

        #code to abort motion if robot gets stuck
        abort_motion_timer += dt
        if abort_motion_timer >= abort_motion_threshold:
            print("Failed to push object. Aborting push.")
            abort_motion()
            return None
        #if robot arm lifts too high, this means the push failed.
        if current_end_effector_position[2] > end_position[2] + 0.015:
            print("Robot finger skidded above object. Aborting push.")
            abort_motion()
            return None

        '''print("pushing end effector position:",current_end_effector_position)
        print("\ttarget:",end_position)
        print("\tdetached contact:",detached_contact)
        print("\tabort motion counter:", str(abort_motion_timer)+"/"+str(abort_motion_threshold))'''

        time.sleep(dt)
    #input("Press enter to continue")

    #if the number of time steps with contact is too small, the push failed
    if len(object_contact_time_step_indices) < 30:
        print("Failed to collect enough contact points when pushing object. Aborting push.")
        abort_motion()
        return None

    #pull up. Also used for aborting motion
    pull_up_position = [0.4, 0., 0.3]
    joint_pos_list = p.calculateInverseKinematics(robotID, 19, pull_up_position)
    for i in np.arange(len(joint_pos_list)):
        p.setJointMotorControl2(robotID, i, controlMode=p.POSITION_CONTROL, targetPosition=joint_pos_list[i])
    time_amount = 0.
    while time_amount < 0.5:
        p.stepSimulation()
        time_amount += dt
        time.sleep(dt)

    #get the angle adjusted robot joint torques #and object pushing forces
    angle_adjusted_robot_joint_torques = []
    #angle_adjusted_object_pushing_forces = []
    for count, index in enumerate(object_contact_time_step_indices):
        sin_angle = np.sin(pushing_angles[count])

        angle_adjusted_robot_joint_torques_single_time_step = []
        for j in np.arange(len(robot_joint_torques[count])):
            angle_adjusted_robot_joint_torques_single_time_step.append(sin_angle * robot_joint_torques[count][j])
        angle_adjusted_robot_joint_torques.append(angle_adjusted_robot_joint_torques_single_time_step)

        #angle_adjusted_object_pushing_forces.append(sin_angle * object_pushing_forces[count])

    push_data = object_contact_points_along_axis, \
                object_pushing_forces, angle_adjusted_robot_joint_torques, \
                actual_pushing_forces, actual_friction_forces, actual_pushing_torques, actual_friction_torques, \
                object_contact_time_step_indices, \
                object_velocities_x, object_velocities_y, object_angular_velocities

    #output feature vectors
    samples_inputs = []
    samples_outputs = []
    for i in np.arange(len(object_contact_time_step_indices)):
        input_features = []
        input_features += angle_adjusted_robot_joint_torques[i][1:2]

        samples_inputs.append(input_features)
        samples_outputs.append(object_pushing_forces[i])

    return push_data, samples_inputs, samples_outputs




def process_push_data(push_data, current_test_dir, push_num):
    #create folder for push data
    data_folder = os.path.join(current_test_dir, "push_"+str(push_num))
    os.mkdir(data_folder)

    push_data_file = open(os.path.join(data_folder, "push_data.csv"), 'w')
    push_data_file.write(", " +
                         "contact points along object axis, pushing forces\n")

    object_contact_points_along_axis, \
                object_pushing_forces, angle_adjusted_robot_joint_torques, \
                actual_pushing_forces, actual_friction_forces, actual_pushing_torques, actual_friction_torques, \
                object_contact_time_step_indices, \
                object_velocities_x, object_velocities_y, object_angular_velocities = push_data

    cutoff = 5
    object_contact_points_along_axis = object_contact_points_along_axis[cutoff:]
    object_pushing_forces = object_pushing_forces[cutoff:]
    #angle_adjusted_robot_joint_torques = angle_adjusted_robot_joint_torques[cutoff:]
    actual_pushing_forces = actual_pushing_forces[cutoff:]
    actual_friction_forces = actual_friction_forces[cutoff:]
    actual_pushing_torques = actual_pushing_torques[cutoff:]
    actual_friction_torques = actual_friction_torques[cutoff:]
    object_contact_time_step_indices = object_contact_time_step_indices[cutoff:]
    #object_velocities_x = object_velocities_x[cutoff:]
    #object_velocities_y = object_velocities_y[cutoff:]
    #object_angular_velocities = object_angular_velocities[cutoff:]

    num_lines = len(object_contact_points_along_axis)
    for i in np.arange(num_lines):
        push_data_file.write(
                             str(object_contact_points_along_axis[i])+","+str(object_pushing_forces[i])+"\n")
    push_data_file.close()


    object_parameter_estimates_file = open(os.path.join(data_folder, "object_parameter_estimates.txt"), 'w')


    forces_times_contact_points_along_axis = []
    for i in np.arange(num_lines):
        forces_times_contact_points_along_axis.append(object_contact_points_along_axis[i] * object_pushing_forces[i])
    angular_velocity_regression_inputs = []
    for i in np.arange(num_lines):
        angular_velocity_regression_inputs.append([forces_times_contact_points_along_axis[i], object_pushing_forces[i]])


    print(object_contact_points_along_axis)
    print(angular_velocity_regression_inputs)


    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)
    axis_vector = get_object_axes()[0]

    min_pos = get_object_link_position(min_main_axis_link_index)
    actual_com_along_axis = np.dot(actual_com - min_pos, axis_vector)

    print()

    print("actual_I:",actual_I)
    print("actual_com_along_axis:",actual_com_along_axis)
    print()



    forces_times_contact_points_along_axis_actual = []
    for i in np.arange(num_lines):
        forces_times_contact_points_along_axis_actual.append((object_contact_points_along_axis[i] - actual_com_along_axis) * object_pushing_forces[i])




    draw_data.plt.title("time step vs actual_friction_torques")
    draw_data.plt.xlabel("time step")
    draw_data.plt.ylabel("actual_friction_torques")
    draw_data.plt.plot([i for i in range(len(actual_friction_torques))], actual_friction_torques, 'ob')
    draw_data.plt.show()


    draw_data.plt.title("time step vs actual_friction_forces")
    draw_data.plt.xlabel("time step")
    draw_data.plt.ylabel("actual_friction_forces")
    draw_data.plt.plot([i for i in range(len(actual_friction_forces))], actual_friction_forces, 'ob')
    draw_data.plt.show()


    draw_data.plt.title("time step vs object_angular_velocities")
    draw_data.plt.xlabel("time step")
    draw_data.plt.ylabel("object_angular_velocities")
    draw_data.plt.plot([i for i in range(len(object_angular_velocities))], object_angular_velocities, 'ob')
    draw_data.plt.plot([i for i in object_contact_time_step_indices], [object_angular_velocities[i] for i in object_contact_time_step_indices], 'or')
    draw_data.plt.show()


    exit()




actual_mass = actual_com = actual_I = None
actual_com_object_frame = None
objectID = None
dt = 1./240.
object_scale = 0.015
startPos = (0.4, 0, 1.*object_scale)
startOrientation = p.getQuaternionFromEuler([np.pi / 2, 0, 0])#p.getQuaternionFromEuler([np.pi / 2, 0, np.pi / 2])
num_links = None

object_original_main_axis = np.array([-1.,0.,0.])
object_original_second_axis = np.array([0.,0.,1.])

object_main_axis_extreme_link_indices = None
object_second_axis_extreme_link_indices = None

min_main_axis_link_index = None
max_main_axis_link_index = None
min_second_axis_link_index = None
max_second_axis_link_index = None

abort_motion_threshold = 2.

def run_full_test(object_name):
    # make directory for simulation files
    test_num = 1
    while os.path.exists("test" + str(test_num)):
        test_num += 1
    test_dir = "test" + str(test_num)
    os.mkdir(test_dir)

    #make the file being tested
    make_URDF.write_URDF(object_name, test_dir)

    #physicsClient = p.connect(p.DIRECT)
    physicsClient = p.connect(p.GUI)
    p.setGravity(0,0,-9.8)

    global objectID
    global num_links
    objectID = p.loadURDF(test_dir + "\\" + object_name + ".urdf", globalScaling=object_scale)
    num_links = p.getNumJoints(objectID)  # excludes base

    #scale the object's mass
    mass_scale = 10.
    p.changeDynamics(objectID, -1, mass_scale * p.getDynamicsInfo(objectID, -1)[0])
    for linkID in np.arange(num_links):
        p.changeDynamics(objectID, linkID, mass_scale * p.getDynamicsInfo(objectID, linkID)[0])

    print("object base friction:",p.getDynamicsInfo(objectID, -1)[1])
    for linkID in np.arange(p.getNumJoints(objectID)):
        print("object link "+str(linkID) + " friction:",p.getDynamicsInfo(objectID, linkID)[1])

    global min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index
    min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index = get_extreme_links_based_on_object_axes()

    #load plane and robot arm
    global planeID
    planeID = p.loadURDF("plane.urdf", useFixedBase=True)
    p.changeVisualShape(planeID, -1, rgbaColor=(0., 0., 0.9, 1.))
    global robotID
    robotID = p.loadURDF("robot_arm_combined\\robot_arm_combined.urdf", useFixedBase=True)#("kuka_lwr\\kuka.urdf")
    #gripperID = p.loadURDF("robotiq_gripper\\robotiq-3f-gripper_articulated.urdf")

    print("object:",objectID)
    print("plane:",planeID)
    print("robot arm:",robotID)
    #print("gripper:",gripperID)

    #print joint info
    num_joints_robot_arm = p.getNumJoints(robotID)
    print("Num Joints:",num_joints_robot_arm)
    for joint_id in np.arange(num_joints_robot_arm):
        print("Joint info for "+str(joint_id)+":", p.getJointInfo(robotID,joint_id))

    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)

    global actual_mass
    global actual_com
    global actual_I
    actual_mass, actual_com, actual_I = get_actual_mass_com_and_moment_of_inertia() #global values
    print("actual_mass, actual_com, actual_I:",actual_mass, actual_com, actual_I)
    global actual_com_object_frame
    tr = transformation_matrix(np.array(list(p.getMatrixFromQuaternion(startOrientation))).reshape((3,3)).T, -1*np.array(list(startPos)))
    actual_com_object_frame = np.matmul(tr, np.append(actual_com,[1]))[:3]
    print("actual_com_object_frame",actual_com_object_frame)



    #all_push_data = []
    all_samples_inputs = []
    all_samples_outputs = []

    num_pushes = 5
    for i in np.arange(num_pushes):
        print("push #"+str(i+1)+"/"+str(num_pushes))
        time.sleep(1.)
        push_result = None
        push_abort_countdown = 5
        while push_result is None:
            #get position of geometric center
            current_position = np.array(list(p.getBasePositionAndOrientation(objectID)[0]))
            for j in np.arange(num_links):
                current_position += p.getLinkState(objectID, j)[0]
            current_position /= num_links + 1.

            #if object is too close or too far to be reached, reset the simulation
            approximate_object_to_robot_distance = np.linalg.norm(current_position)
            if approximate_object_to_robot_distance < 0.3 or approximate_object_to_robot_distance > 0.8:
                print("Object in unreachable configuration. Resetting.")
                time.sleep(1.)
                p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)

            #determine if to move closer or further from robot
            motion_toward_robot = True
            if np.linalg.norm(current_position) < .5:
                motion_toward_robot = False

            #push
            finger_start_pos, finger_end_pose = get_push_start_and_end_positions(0, motion_toward_robot)
            push_result = push_object(finger_start_pos, finger_end_pose, 0)

            #collect data from push if successful. If push is unsuccessful, reduce push abort countdown by 1
            if push_result is not None:
                push_data, single_push_samples_inputs, single_push_samples_outputs = push_result
                all_samples_inputs += single_push_samples_inputs
                all_samples_outputs += single_push_samples_outputs

                process_push_data(push_data, test_dir, i+1)
                #all_push_data.append(push_data)
            else:
                push_abort_countdown -= 1

            #if three pushes fail, reset the simulation and try again
            if push_abort_countdown == 0:
                print("Object in unreachable configuration. Resetting.")
                time.sleep(1.)
                p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)

    '''all_push_data_to_process = []
    for i in np.arange(len(all_push_data[0])):
        all_push_data_to_process.append([])
    for i in np.arange(len(all_push_data)):
        for j in np.arange(len(all_push_data[i])):
            all_push_data_to_process[j] += all_push_data[i][j]
    process_push_data(all_push_data_to_process, test_dir)#, i + 1)'''


    p.disconnect()

    #handle the data
    '''kernel = gp.kernels.ConstantKernel(1.0, (1e-1, 1e3)) * gp.kernels.RBF(10.0, (1e-3, 1e4))
    model = gp.GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10, alpha=0.1, normalize_y=True)
    #model = lin_mod.LinearRegression()
    gaussian = True

    all_samples_inputs = np.array(all_samples_inputs)
    all_samples_outputs = np.array(all_samples_outputs)

    print(all_samples_inputs)
    print(all_samples_outputs)

    print(all_samples_inputs.shape)
    print(all_samples_outputs.shape)

    input_train, input_test, output_train, output_test = mod_sel.train_test_split(all_samples_inputs, all_samples_outputs, test_size=0.5)

    print(input_train.shape)
    print(output_train.shape)

    print(input_test.shape)
    print(output_test.shape)

    model.fit(input_train, output_train)

    output_pred = None
    output_pred_std = None
    if gaussian:
        output_pred, output_pred_std = model.predict(input_test, return_std=True)
    else:
        output_pred = model.predict(input_test)

    print("ground truth:\t", output_test)
    print("results:\t\t", output_pred)
    if gaussian:
        print("result std devs:\t", output_pred_std)

    draw_data.plt.title("forces: ground truth vs predictions")
    draw_data.plt.xlabel("ground truth")
    draw_data.plt.ylabel("predictions")
    if gaussian:
        draw_data.plt.errorbar(output_test, output_pred, yerr=output_pred_std, fmt='ro')
    else:
        draw_data.plt.plot(output_test, output_pred, 'ro')
    draw_data.plt.plot(output_test, output_test, 'b-')
    draw_data.plt.show()

    print()
    print("Model parameters:")
    if gaussian:
        print(model.X_train_,"\n",model.y_train_,"\n",model.kernel_,"\n",model.L_,"\n",model.alpha_,"\n",model.log_marginal_likelihood_value_)
    else:
        print(model.coef_,"\n",model.intercept_)'''

    print("Done.")


run_full_test("spray_gun_uniform")
'''run_full_test("spray_gun")
run_full_test("snack_uniform")
run_full_test("snack")
run_full_test("book_uniform")
run_full_test("book")
run_full_test("wrench_uniform")
run_full_test("wrench")
run_full_test("hammer_uniform")
run_full_test("hammer")'''
