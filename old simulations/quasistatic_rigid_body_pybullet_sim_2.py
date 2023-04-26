import random

import pybullet as p
import numpy as np
import draw_data
import make_URDF

import os
import time

import sklearn.linear_model as lin_mod




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





def get_actual_mass_com_cof_and_moment_of_inertia():
    #count = 1
    masses = []
    mass, friction = p.getDynamicsInfo(objectID, -1)[:2] #base mass and friction
    mass_times_friction = mass * friction
    masses.append(mass)

    loc = np.array(p.getBasePositionAndOrientation(objectID)[0]) #base location
    loc_weighed_mass = masses[0] * loc
    loc_weighed_mass_times_friction = mass_times_friction * loc

    for i in range(num_links):
        this_mass, this_friction = p.getDynamicsInfo(objectID, i)[:2]
        masses.append(this_mass)
        mass += this_mass
        this_mass_times_friction = this_mass*this_friction
        mass_times_friction += this_mass_times_friction

        this_loc = np.array(p.getLinkState(objectID, i)[0])
        loc_weighed_mass += this_loc * this_mass
        loc_weighed_mass_times_friction += this_loc * this_mass_times_friction

        #count += 1
    com = loc_weighed_mass/mass
    cof = loc_weighed_mass_times_friction/mass_times_friction

    object_scale_factor = object_scale**2
    I = object_scale_factor*masses[0]/6. + masses[0]*(np.linalg.norm(np.array(p.getBasePositionAndOrientation(objectID)[0]) - com)**2)
    for i in range(num_links):
        #print("I",I)
        I += object_scale_factor*masses[i+1]/6. + masses[i+1]*(np.linalg.norm(np.array(p.getLinkState(objectID, i)[0]) - com) ** 2)


    '''other_I = p.getDynamicsInfo(objectID,-1)[2][1] + masses[0]*(np.linalg.norm(np.array(p.getBasePositionAndOrientation(objectID)[0]) - com)**2)
    for i in range(num_links):
        print("other I", other_I,p.getDynamicsInfo(objectID,i)[2])
        other_I += p.getDynamicsInfo(objectID,i)[2][1] + masses[i+1]*(np.linalg.norm(np.array(p.getLinkState(objectID, i)[0]) - com) ** 2)
    print("\n\n\n")
    print("object_scale",object_scale)
    print("I",I)
    print("other I",other_I)
    exit()'''

    return mass, com, cof, I





def five_point_central_difference(index, series):
    #index should be at least 2 and at most len(series)-3
    return (series[index-2] - 8*series[index-1] + 8*series[index+1] - series[index+2]) / (12.*dt)





def transformation_matrix(R, pos):
    return np.block([
        [R, pos.reshape(3,1)],
        [np.array([[0., 0., 0., 1.]])]
    ])




def get_point_on_object_in_world_coordinates(original_point):
    position, orientation = p.getBasePositionAndOrientation(objectID)
    r = np.array(p.getMatrixFromQuaternion(orientation)).reshape((3,3))
    tr = transformation_matrix(r, np.array(position))
    return np.matmul(tr, np.append(original_point, [1]))[:3]




def get_extreme_links_based_on_object_axes():
    #NOTE: for this method to work, object must be in its initial position and orientation

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





'''def get_link_along_axes(position_on_axis, axis, other_axis):
    #NOTE: for this method to work, object must be in its initial position and orientation

    if axis is object_original_main_axis:
        min_point_on_axis = np.dot(get_object_link_position(min_main_axis_link_index), axis)
    else:
        min_point_on_axis = np.dot(get_object_link_position(min_second_axis_link_index), axis)
    position_on_axis = min_point_on_axis + position_on_axis

    indices = []
    locs = []

    loc = np.array(p.getBasePositionAndOrientation(objectID)[0])
    loc_on_axis = np.dot(loc, axis)
    dist = np.abs(position_on_axis - loc_on_axis)
    indices.append(-1)
    locs.append(loc)

    #get indices of link closest to position specified
    for i in range(num_links):
        loc = np.array(p.getLinkState(objectID, i)[0])
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
    return final_index'''





def get_rotated_object_axis(axis):
    object_basic_r = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(objectID)[1])).reshape((3,3))
    return np.matmul(object_basic_r, axis)




def get_point_on_axis(point_world_coords, axis_index):
    original_axis = object_original_main_axis
    if axis_index > 0:
        original_axis = object_original_second_axis

    axis = get_rotated_object_axis(original_axis)

    if original_axis is object_original_main_axis:
        min_point_on_axis = np.dot(get_object_link_position(min_main_axis_link_index), axis)
    else:
        min_point_on_axis = np.dot(get_object_link_position(min_second_axis_link_index), axis)

    return np.dot(point_world_coords, axis) - min_point_on_axis





def push_object(pusher_end, out_dir):

    #get the video
    loggerID = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.path.join(out_dir, "video.mp4"))

    test_point_velocities = []
    angles = []
    angular_velocities = []
    pushing_forces = []
    pushing_forces_on_axis = []
    pushing_contact_points_on_axis = []

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
            pushing_contact_point_on_axis = get_point_on_axis(pushing_contact_point, 0)
            pushing_contact_points_on_axis.append(pushing_contact_point_on_axis)
        else:
            if made_contact:
                broke_contact_countdown -= 1
                if broke_contact_countdown == 0:
                    broke_contact = True
            pushing_contact_points_on_axis.append(None)
        pushing_forces.append(contact_force)
        object_second_axis = get_rotated_object_axis(object_original_second_axis)
        contact_force_on_axis = np.dot(contact_force, object_second_axis)
        pushing_forces_on_axis.append(contact_force_on_axis)

        #reset the pusher
        pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
        p.resetBasePositionAndOrientation(pusherID, pusher_position, (0.,0.,0.,1.))
        pusher_displacement_from_destination = pusher_end - np.array(pusher_position)
        pusher_dist_from_destination = np.linalg.norm(pusher_displacement_from_destination)
        pusher_speed = .15
        new_pusher_velocity = pusher_speed * pusher_displacement_from_destination/pusher_dist_from_destination
        p.resetBaseVelocity(pusherID, (new_pusher_velocity[0], new_pusher_velocity[1], new_pusher_velocity[2]), (0., 0., 0.))
        p.applyExternalForce(pusherID, -1, (0., 0., 9.8), (0.,0.,0.), p.LINK_FRAME) #antigravity


    p.stopStateLogging(loggerID)

    return angles, test_point_velocities, angular_velocities, pushing_forces, pushing_forces_on_axis, pushing_contact_points_on_axis




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




def plot_variables(var1, var1_name, var2, var2_name, out_dir, title_preamble, line_data=None, show=False):
    title = title_preamble + var1_name + " vs "+ var2_name
    draw_data.plt.title(title)
    draw_data.plt.xlabel(var1_name)
    draw_data.plt.ylabel(var2_name)
    for i in np.arange(len(var1)):
        draw_data.plt.plot(var1[i], var2[i], marker='o', color=(1.-(i/len(var1)),0.,i/len(var1),1.))
    if line_data is not None:
        draw_data.plt.plot(var1,line_data, 'r-')
    draw_data.plt.savefig(os.path.join(out_dir, title))
    if show:
        draw_data.plt.show()
    draw_data.plt.close("all")




actual_mass = actual_com = actual_cof = actual_I = None
objectID = None
pusherID = None
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

    '''print("object base friction:",p.getDynamicsInfo(objectID, -1)[1])
    for linkID in np.arange(p.getNumJoints(objectID)):
        print("object link "+str(linkID) + " friction:",p.getDynamicsInfo(objectID, linkID)[1])'''

    global min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index
    links, distances_on_axes = get_extreme_links_based_on_object_axes()
    min_main_axis_link_index, max_main_axis_link_index, min_second_axis_link_index, max_second_axis_link_index = links

    #set up the camera
    set_up_camera(startPos)

    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)

    #get the indices of the links to be used in the push attempts
    axis_length = distances_on_axes[1] - distances_on_axes[0]
    points_to_push_on_axis = np.linspace(0, axis_length, 7)[1:-1]
    object_basic_main_axis = get_rotated_object_axis(object_original_main_axis)
    object_basic_second_axis = get_rotated_object_axis(object_original_second_axis)
    pusher_points = []
    print("object_basic_main_axis:",object_basic_main_axis)
    print("object_basic_second_axis",object_basic_second_axis)
    for point_to_push in points_to_push_on_axis:
        pusher_start = get_object_link_position(min_main_axis_link_index) + point_to_push*object_basic_main_axis + 0.15*object_basic_second_axis + np.array([0.,0.,0.05])
        pusher_end = pusher_start - 0.3*object_basic_second_axis
        pusher_points.append((pusher_start,pusher_end))
    print(pusher_points)

    #get actual mass distribution moments
    global actual_mass
    global actual_com
    global actual_cof
    global actual_I
    actual_mass, actual_com, actual_cof, actual_I = get_actual_mass_com_cof_and_moment_of_inertia() #global values
    print("actual_mass, actual_com, actual_I:",actual_mass, actual_com, actual_I)
    print("actual cof:", actual_cof)

    for i,pusher_start_end_pair in enumerate(pusher_points):
        pusher_start, pusher_end = pusher_start_end_pair

        push_file_dir = os.path.join(test_dir, "push_"+str(i))
        os.mkdir(push_file_dir)

        p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)
        p.resetBasePositionAndOrientation(pusherID, (pusher_start[0], pusher_start[1], pusher_start[2]), (0., 0., 0., 1.))

        angles, test_point_velocities, angular_velocities, pushing_forces, pushing_forces_on_axis, pushing_contact_points_on_axis = push_object(pusher_end, push_file_dir)

        plot_variable_vs_time(angles, "angle", push_file_dir)
        plot_variable_vs_time(test_point_velocities, "velocity of test point", push_file_dir)
        plot_variable_vs_time(angular_velocities, "angular velocity", push_file_dir)
        #plot_variable_vs_time(pushing_forces, "pushing force", push_dir)

        push_data_file = open(os.path.join(push_file_dir, "push_data.csv"), "w")
        push_data_file.write("time,angle,angular velocity,contact point on object axis,pushing force x,pushing force y, pushing force on object axis\n")
        for j in np.arange(len(angles)):
            contact_point_str = ""
            if pushing_contact_points_on_axis[j] is not None:
                contact_point_str = str(pushing_contact_points_on_axis[j])
            push_data_file.write(str(j*dt)+","+str(angles[j])+","+str(angular_velocities[j])+","+ contact_point_str
                                 +","+str(pushing_forces[j][0])+","+str(pushing_forces[j][1])+","+str(pushing_forces_on_axis[j])+"\n")
        push_data_file.close()

        p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)
        p.resetBasePositionAndOrientation(pusherID, (pusher_start[0], pusher_start[1], pusher_start[2]), (0., 0., 0., 1.))

        actual_com_on_axis = get_point_on_axis(actual_com, 0)
        print("actual r_com", actual_com_on_axis)
        actual_cof_on_axis = get_point_on_axis(actual_cof, 0)
        print("actual r_cof", actual_cof_on_axis)

        #quasi-static assumption: F_secondAxis*(r_mainAxis - r_com) - tau_f = 0, so F_secondAxis r_mainAxis = tau_f + F_secondAxis r_com
        r_and_f_incides = []
        r_on_main_axis = []
        f_on_second_axis = []
        for j in np.arange(len(angles)):
            if pushing_contact_points_on_axis[j] is not None:
                r_and_f_incides.append(j)
                r_on_main_axis.append(pushing_contact_points_on_axis[j])
                f_on_second_axis.append(pushing_forces_on_axis[j])
        #offset
        offset = 20
        r_and_f_incides = r_and_f_incides[offset:-offset]
        r_on_main_axis = r_on_main_axis[offset:-offset]
        f_on_second_axis = f_on_second_axis[offset:-offset]
        f_times_r = []
        for j in np.arange(len(r_on_main_axis)):
            f_times_r.append(r_on_main_axis[j]*f_on_second_axis[j])

        #do linear regression
        model_lin = lin_mod.LinearRegression()
        model_lin.fit(np.array(f_on_second_axis).reshape((-1,1)), f_times_r)
        line_data = model_lin.predict(np.array(f_on_second_axis).reshape((-1,1)))
        plot_variables(f_on_second_axis, "force second axis", f_times_r, "f_times_r", push_file_dir, "Linear reg - ", line_data=line_data)

        #do regression usign RANSAC
        model_RANSAC = lin_mod.RANSACRegressor()
        model_RANSAC.fit(np.array(f_on_second_axis).reshape((-1,1)), f_times_r)
        line_data = model_RANSAC.predict(np.array(f_on_second_axis).reshape((-1,1)))
        plot_variables(f_on_second_axis, "force second axis", f_times_r, "f_times_r", push_file_dir, "RANSAC reg - ", line_data=line_data)

        #non quasi-static. F_secondAxis*(r_mainAxis - r_com) - tau_f = I alpha, so F_secondAxis r_mainAxis - I alpha = tau_f + F_secondAxis r_com
        #this is to see if deviations from the correct COM can be accounted for by deviations from the quasi-static assumption
        angular_accelerations = []
        for j in r_and_f_incides:
            if j < len(angles)-2:
                angular_accelerations.append(five_point_central_difference(j, angular_velocities))
        while len(angular_accelerations) < len(r_and_f_incides):
            r_and_f_incides.pop()
            f_times_r.pop()
            f_on_second_axis.pop()
        f_times_r_minus_I_alpha = []
        for j in np.arange(len(f_times_r)):
            f_times_r_minus_I_alpha.append(f_times_r[j] - actual_I*angular_accelerations[j])
        model_lin_non_quasistatic = lin_mod.LinearRegression()
        model_lin_non_quasistatic.fit(np.array(f_on_second_axis).reshape((-1,1)), f_times_r_minus_I_alpha)
        line_data = model_lin_non_quasistatic.predict(np.array(f_on_second_axis).reshape((-1,1)))
        plot_variables(f_on_second_axis, "force second axis", f_times_r_minus_I_alpha, "f_times_r_minus_I_alpha", push_file_dir, "Linear reg - acc adjusted - ", line_data=line_data)

        results_file = open(os.path.join(push_file_dir, "results_file.txt"), "w")
        results_file.write("r_com values are points on the object's main axis, defined as an offset from one end of the object.\n\n")
        results_file.write("Lin reg estimated tau_f:\t" + str(model_lin.intercept_)+"\n")
        results_file.write("Lin reg estimated r_com:\t" + str(model_lin.coef_[0])+"\n\n")
        results_file.write("RANSAC estimated tau_f:\t" + str(model_RANSAC.estimator_.intercept_)+"\n")
        results_file.write("RANSAC estimated r_com:\t" + str(model_RANSAC.estimator_.coef_[0])+"\n\n")
        results_file.write("Angular acceleration adjusted lin reg estimated tau_f:\t" + str(model_lin_non_quasistatic.intercept_)+"\n")
        results_file.write("Angular acceleration adjusted lin reg estimated r_com:\t" + str(model_lin_non_quasistatic.coef_[0])+"\n\n")
        results_file.write("actual r_com:\t" + str(actual_com_on_axis)+"\n")
        results_file.write("actual r_cof:\t" + str(actual_cof_on_axis)+"\n")
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