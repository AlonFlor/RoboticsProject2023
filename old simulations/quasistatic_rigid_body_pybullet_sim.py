import random

import pybullet as p
import numpy as np
import draw_data
import make_URDF

import os
import time


def set_up_camera(pos):
    #from https://github.com/changkyu/iros2020_changkyu/blob/master/software/simulator/SimBullet.py

    distance = 0.75
    yaw = 45
    pitch = -35.
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=pos,
                                        distance=distance,
                                        yaw=yaw,
                                        pitch=pitch,
                                        roll=0,
                                        upAxisIndex=2)
    proj_matrix = p.computeProjectionMatrixFOV(fov=53.130,
                                 aspect= 640. / 480.,
                                 nearVal=0.001,
                                 farVal=100.0)

    p.resetDebugVisualizerCamera(distance, yaw, pitch, pos)

    return view_matrix, proj_matrix





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

    return (min_main_axis[0], max_main_axis[0], min_second_axis[0], max_second_axis[0]), (min_main_axis[1], max_main_axis[1], min_second_axis[1], max_second_axis[1])





def get_object_link_position(index):
    if index != -1:
        return np.array(list(p.getLinkState(objectID, index)[0]))
    else:
        return np.array(list(p.getBasePositionAndOrientation(objectID)[0]))





def get_link_along_axes(position_on_axis, axis, other_axis):
    #NOTE: for this method to work, object must be in its initial position and orientation

    if axis is object_original_main_axis:
        min_point_on_axis = np.dot(get_object_link_position(min_main_axis_link_index), axis)
    else:
        min_point_on_axis = np.dot(get_object_link_position(min_second_axis_link_index), axis)
    position_on_axis = min_point_on_axis + position_on_axis

    indices = []
    locs = []

    loc = np.array(list(p.getBasePositionAndOrientation(objectID)[0]))
    loc_on_axis = np.dot(loc, axis)
    dist = np.abs(position_on_axis - loc_on_axis)
    indices.append(-1)
    locs.append(loc)

    #get indices of link closest to position specified
    for i in range(num_links):
        loc = np.array(list(p.getLinkState(objectID, i)[0]))
        loc_on_axis = np.dot(loc, axis)
        new_dist = np.abs(position_on_axis - loc_on_axis)
        if new_dist < dist:
            indices.clear()
            indices.append(i)
            locs.clear()
            locs.append(loc)
            dist = new_dist
        elif new_dist == dist:
            indices.append(i)
            locs.append(loc)

    #get index of link closest to edge of object along second axis
    final_index = indices[0]
    best_loc_on_other_axis = np.dot(locs[0], other_axis)
    for i in np.arange(len(indices)):
        loc = locs[i]
        loc_on_other_axis = np.dot(loc, other_axis)
        if loc_on_other_axis > best_loc_on_other_axis:
            best_loc_on_other_axis = loc_on_other_axis
            final_index = indices[i]
    return final_index




def push_object(link_index, link_original_position, initial_pushing_force, stop_angle, out_dir):

    #get the video
    loggerID = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.path.join(out_dir, "video.mp4"))

    #velocities = []
    angles = []
    angular_velocities = []
    pushing_forces = []

    velocity = np.array([0., 0., 0.])
    angular_velocity = np.array([0., 0., 0.])
    initial_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
    current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]

    #wait beforehand
    time_amount = 0.5
    while time_amount > 0.:
        p.stepSimulation()
        time_amount -= dt
        time.sleep(dt)

        current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
        angles.append(current_angle)

        angular_velocity = p.getBaseVelocity(objectID)[1]
        angular_velocity = np.array(list(angular_velocity))
        angular_velocities.append(angular_velocity[2])

        pushing_forces.append(0.)

    #push until object starts moving
    while np.linalg.norm(velocity) + np.linalg.norm(angular_velocity) < 1.5:
        force_position = get_point_on_object_in_world_coordinates(link_original_position)
        p.applyExternalForce(objectID, link_index, (0., initial_pushing_force, 0.), force_position, p.WORLD_FRAME)

        current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
        angles.append(current_angle)

        velocity, angular_velocity = p.getBaseVelocity(objectID)
        velocity = np.array(list(velocity))
        angular_velocity = np.array(list(angular_velocity))
        angular_velocities.append(angular_velocity[2])

        pushing_forces.append(initial_pushing_force)

        p.stepSimulation()
        time.sleep(dt)

    #push to resist friction
    while np.abs(current_angle - initial_angle) < stop_angle:

        friction_contacts = p.getContactPoints(objectID, planeID)
        friction_force = np.array([0., 0., 0.])
        for friction_contact in friction_contacts:
            friction_force += friction_contact[10]*np.array(list(friction_contact[11])) + friction_contact[12]*np.array(list(friction_contact[13]))
        friction_force_magn = np.linalg.norm(friction_force)
        #print("friction_force_magn", friction_force_magn)

        force_position = get_point_on_object_in_world_coordinates(link_original_position)
        p.applyExternalForce(objectID, link_index, (0., friction_force_magn, 0.), force_position, p.WORLD_FRAME)

        current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
        angles.append(current_angle)

        velocity, angular_velocity = p.getBaseVelocity(objectID)
        #velocity = np.linalg.norm(np.array(list(velocity)))
        angular_velocity = np.array(list(angular_velocity))
        angular_velocities.append(angular_velocity[2])
        #print("velocity =",velocity,"angular velocity =",angular_velocity)

        pushing_forces.append(friction_force_magn)

        p.stepSimulation()
        #time_amount -= dt
        time.sleep(dt)

        #velocities.append(velocity)

    #wait afterwards
    time_amount = 0.5
    while time_amount > 0.:
        p.stepSimulation()
        time_amount -= dt
        time.sleep(dt)

        current_angle = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])[2]
        angles.append(current_angle)

        angular_velocity = p.getBaseVelocity(objectID)[1]
        angular_velocity = np.array(list(angular_velocity))
        angular_velocities.append(angular_velocity[2])

        pushing_forces.append(0.)

    p.stopStateLogging(loggerID)

    return angles, angular_velocities, pushing_forces




def plot_variable_vs_time(variable, variable_name, out_dir, show=False):
    title = "time vs "+ variable_name
    draw_data.plt.title(title)
    draw_data.plt.xlabel("time")
    draw_data.plt.ylabel(variable_name)
    draw_data.plt.plot([i*dt for i in range(len(variable))], variable, 'ob')
    draw_data.plt.savefig(os.path.join(out_dir, title))
    if show:
        draw_data.plt.show()
    draw_data.plt.close("all")




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

    #load plane
    global planeID
    planeID = p.loadURDF("plane.urdf", useFixedBase=True)
    p.changeVisualShape(planeID, -1, rgbaColor=(0., 0., 0.9, 1.))

    global objectID
    global num_links
    objectID = p.loadURDF(test_dir + "\\" + object_name + ".urdf", globalScaling=object_scale)
    num_links = p.getNumJoints(objectID)  # excludes base

    print("object:", objectID)
    print("plane:", planeID)

    #scale the object's mass
    mass_scale = .1
    p.changeDynamics(objectID, -1, mass_scale * p.getDynamicsInfo(objectID, -1)[0])
    for linkID in np.arange(num_links):
        p.changeDynamics(objectID, linkID, mass_scale * p.getDynamicsInfo(objectID, linkID)[0])

    '''print("object base friction:",p.getDynamicsInfo(objectID, -1)[1])
    for linkID in np.arange(p.getNumJoints(objectID)):
        print("object link "+str(linkID) + " friction:",p.getDynamicsInfo(objectID, linkID)[1])'''

    global min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index
    links, distances_on_axes = get_extreme_links_based_on_object_axes()
    min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index = links

    #set up the camera
    set_up_camera(startPos)

    #get the indices of the links to be used in the push attempts
    axis_length = distances_on_axes[1] - distances_on_axes[0]
    points_to_push_on_axis = np.linspace(0, axis_length, 5)
    links_to_push = []
    links_original_positions = []
    for point_to_push in points_to_push_on_axis:
        link_index = get_link_along_axes(point_to_push, object_original_main_axis, object_original_second_axis)
        links_to_push.append(link_index)
        links_original_positions.append(get_object_link_position(link_index))


    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)

    #get actual mass distribution moments
    global actual_mass
    global actual_com
    global actual_I
    actual_mass, actual_com, actual_I = get_actual_mass_com_and_moment_of_inertia() #global values
    print("actual_mass, actual_com, actual_I:",actual_mass, actual_com, actual_I)
    global actual_com_object_frame
    tr = transformation_matrix(np.array(list(p.getMatrixFromQuaternion(startOrientation))).reshape((3,3)).T, -1*np.array(list(startPos)))
    actual_com_object_frame = np.matmul(tr, np.append(actual_com,[1]))[:3]
    print("actual_com_object_frame",actual_com_object_frame)

    for i in np.arange(len(points_to_push_on_axis)):
        push_dir = os.path.join(test_dir, "push_"+str(i))
        os.mkdir(push_dir)

        p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)

        p.changeVisualShape(objectID, links_to_push[i], rgbaColor=(0., 1., 0., 1.))
        initial_pushing_force = 150.
        stop_angle = np.pi / 3.
        angles, angular_velocities, pushing_forces = push_object(links_to_push[i], links_original_positions[i], initial_pushing_force, stop_angle, push_dir)

        plot_variable_vs_time(angles, "angle", push_dir)
        plot_variable_vs_time(angular_velocities, "angular velocity", push_dir)
        plot_variable_vs_time(pushing_forces, "pushing force", push_dir)

        point_to_push_on_axis_file = open(os.path.join(push_dir, "push distance from axis end.txt"), "w")
        point_to_push_on_axis_file.write(str(points_to_push_on_axis[i]))
        point_to_push_on_axis_file.close()

        push_data_file = open(os.path.join(push_dir, "push_data.csv"), "w")
        push_data_file.write("time,angle,angular velocity,pushing force\n")
        for j in np.arange(len(angles)):
            push_data_file.write(str(j*dt)+","+str(angles[j])+","+str(angular_velocities[j])+","+str(pushing_forces[j])+"\n")
        push_data_file.close()

        p.changeVisualShape(objectID, links_to_push[i], rgbaColor=(1., 1., 1., 1.))

    p.disconnect()

run_full_test("spray_gun_uniform")