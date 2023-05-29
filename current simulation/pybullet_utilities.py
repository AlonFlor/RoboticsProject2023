import pybullet as p
import numpy as np
import os

import file_handling
import make_URDF
from PIL import Image
#import cv2


def load_object(name, test_dir, model_COM=(0.,0.,0.), useFixedBase=False):
    i=1
    extra_designation = ""
    while os.path.exists(os.path.join(test_dir,name+extra_designation+".urdf")):
        extra_designation = "_" + str(i).zfill(3)
        i+=1
    make_URDF.write_URDF_from_mesh(name, test_dir, model_COM, extra_designation)
    objectID = p.loadURDF(os.path.join(test_dir,name+extra_designation+".urdf"), useFixedBase=useFixedBase)

    return objectID, [name,model_COM]


#def load_robot()



def set_up_camera(pos, distance=0.75, yaw=45, pitch=-35):
    #from https://github.com/changkyu/iros2020_changkyu/blob/master/software/simulator/SimBullet.py

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


def print_image(view_matrix, proj_matrix, imgs_dir, image_num):
    image_result = p.getCameraImage(640, 480, view_matrix, proj_matrix, renderer=p.ER_TINY_RENDERER, shadow=1)
    w, h, pixels = image_result[:3]
    img_numpy = np.array(pixels).reshape((h, w, 4))

    img = Image.fromarray(img_numpy, "RGBA")
    image_filename = os.path.join(imgs_dir, str(image_num).zfill(4) + ".png")
    img.save(image_filename)


def make_video(test_dir,imgs_dir):
    video_path = os.path.join(test_dir,"video.mp4")
    command = "ffmpeg.exe -framerate 24 -i " + os.path.join(imgs_dir,"%04d.png") + " -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p " + video_path
    os.popen(command)



def get_actual_mass_com_cof_and_moment_of_inertia(objectID, num_links, object_scale=1.):
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




def save_data_one_frame(time_val, fps, mobile_object_IDs, motion_script, image_num, view_matrix, proj_matrix, imgs_dir):
    if (time_val * fps - int(time_val * fps) < 0.0001):
        for ID in mobile_object_IDs:
            add_to_motion_script(ID, time_val, motion_script)

        print_image(view_matrix, proj_matrix, imgs_dir, image_num)
        image_num += 1
    return image_num


def push(pusher_end, pusherID, dt, mobile_object_IDs=None, fps=None, view_matrix=None, proj_matrix=None, imgs_dir=None, available_image_num=None, motion_script=None, time_out=100.):
    count = 0
    image_num = None
    if available_image_num != None:
        image_num = available_image_num + 0
    saving_data = not (mobile_object_IDs==None)

    pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
    while np.linalg.norm(np.array(pusher_position) - pusher_end) > 0.01:
        time_val = count * dt
        if saving_data:
            image_num = save_data_one_frame(time_val, fps, mobile_object_IDs, motion_script, image_num, view_matrix, proj_matrix, imgs_dir)

        if time_val > time_out:
            break #pusher timed out

        p.stepSimulation()

        # reset the pusher
        pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
        pusher_position = (pusher_position[0], pusher_position[1], pusher_end[2])  # keep the height constant
        p.resetBasePositionAndOrientation(pusherID, pusher_position, (0., 0., 0., 1.))
        pusher_displacement_from_destination = pusher_end - np.array(pusher_position)
        pusher_dist_from_destination = np.linalg.norm(pusher_displacement_from_destination)
        pusher_speed = .1
        new_pusher_velocity = pusher_speed * pusher_displacement_from_destination / pusher_dist_from_destination
        p.resetBaseVelocity(pusherID, (new_pusher_velocity[0], new_pusher_velocity[1], new_pusher_velocity[2]), (0., 0., 0.))
        p.applyExternalForce(pusherID, -1, (0., 0., 9.8), (0., 0., 0.), p.LINK_FRAME)  # antigravity

        count += 1

    return image_num



'''def grasp_helper(grasper1_ID, grasper2_ID, grasper_initial_height, graspers_distance, grasper_speed, upward_motion_0_or_1):
    # reset the grapsers
    grasper1_position = p.getBasePositionAndOrientation(grasper1_ID)[0]
    grasper2_position = p.getBasePositionAndOrientation(grasper2_ID)[0]
    if upward_motion_0_or_1==0:
        grasper1_position = (grasper1_position[0], grasper1_position[1], grasper_initial_height)  # keep the height constant
        grasper2_position = (grasper2_position[0], grasper2_position[1], grasper_initial_height)  # keep the height constant
    else:
        grasper2_position = (grasper2_position[0], grasper2_position[1], grasper1_position[2])
    p.resetBasePositionAndOrientation(grasper1_ID, grasper1_position, (0., 0., 0., 1.))
    p.resetBasePositionAndOrientation(grasper2_ID, grasper2_position, (0., 0., 0., 1.))

    #get the vector from grasper2 to grasper1 and get the distance between the graspers
    grasper2_to_grasper1 = np.array(grasper1_position) - np.array(grasper2_position)
    new_graspers_distance = np.linalg.norm(grasper2_to_grasper1)
    graspers_distance_change = new_graspers_distance - graspers_distance
    graspers_distance = new_graspers_distance

    #move the graspers
    new_grasper_velocity = grasper_speed * grasper2_to_grasper1 / graspers_distance
    upward_motion = grasper_speed * upward_motion_0_or_1
    #if upward_motion_0_or_1==1:
    #    new_grasper_velocity*=0.
    p.resetBaseVelocity(grasper1_ID, (-new_grasper_velocity[0], -new_grasper_velocity[1], upward_motion-new_grasper_velocity[2]),(0., 0., 0.))
    p.resetBaseVelocity(grasper2_ID, (new_grasper_velocity[0], new_grasper_velocity[1], upward_motion+new_grasper_velocity[2]),(0., 0., 0.))
    p.applyExternalForce(grasper1_ID, -1, (0., 0., 9.8), (0., 0., 0.), p.LINK_FRAME)  # antigravity
    p.applyExternalForce(grasper2_ID, -1, (0., 0., 9.8), (0., 0., 0.), p.LINK_FRAME)  # antigravity

    return graspers_distance, graspers_distance_change



def grasp(grasper1_ID, grasper2_ID, grasper_initial_height,
          dt, mobile_object_IDs=None, fps=None, view_matrix=None, proj_matrix=None, imgs_dir=None, available_image_num=None, motion_script=None, time_out=100.):
    count = 0
    image_num = None
    if available_image_num != None:
        image_num = available_image_num + 0
    saving_data = not (mobile_object_IDs==None)

    grasper1_position = p.getBasePositionAndOrientation(grasper1_ID)[0]
    grasper2_position = p.getBasePositionAndOrientation(grasper2_ID)[0]
    graspers_distance = np.linalg.norm(np.array(grasper1_position) - np.array(grasper2_position))
    graspers_distance_change = -100.
    time_val = 0.
    grasper_speed = .1

    #clamp the graspers to the object
    grasp_helper(grasper1_ID, grasper2_ID, grasper_initial_height, graspers_distance, grasper_speed, 0)
    while graspers_distance_change < 0.:
        time_val = count * dt
        if saving_data:
            image_num = save_data_one_frame(time_val, fps, mobile_object_IDs, motion_script, image_num, view_matrix, proj_matrix, imgs_dir)

        if time_val > time_out:
            break #pusher timed out

        p.stepSimulation()

        graspers_distance, graspers_distance_change = grasp_helper(grasper1_ID, grasper2_ID, grasper_initial_height, graspers_distance, grasper_speed, 0)
        count += 1

    #lift the object
    while(time_val < time_out):
        time_val = count * dt
        if saving_data:
            image_num = save_data_one_frame(time_val, fps, mobile_object_IDs, motion_script, image_num, view_matrix, proj_matrix, imgs_dir)

        p.stepSimulation()

        grasp_helper(grasper1_ID, grasper2_ID, grasper_initial_height, graspers_distance, grasper_speed, 1)
        count += 1

    return image_num'''



def let_time_pass(time_amount, pusherID, dt, mobile_object_IDs=None, fps=None, view_matrix=None, proj_matrix=None, imgs_dir=None, available_image_num=None, motion_script=None):
    #for pusher
    count=0
    image_num = None
    if available_image_num != None:
        image_num = available_image_num + 0
    saving_data = not (mobile_object_IDs==None)

    while time_amount>0:
        time_val = count * dt
        if saving_data:
            image_num = save_data_one_frame(time_val, fps, mobile_object_IDs, motion_script, image_num, view_matrix, proj_matrix, imgs_dir)
        count += 1

        p.stepSimulation()

        p.resetBaseVelocity(pusherID, (0., 0., 0.), (0., 0., 0.))
        p.applyExternalForce(pusherID, -1, (0., 0., 9.8), (0., 0., 0.), p.LINK_FRAME)  # antigravity

        time_amount -= dt

    return image_num




def quaternion_multiplication(q1, q2):
    r1 = q1[3]
    r2 = q2[3]
    v1 = np.array([q1[0], q1[1], q1[2]])
    v2 = np.array([q2[0], q2[1], q2[2]])
    v_ans = r1*v2 + r2*v1 + np.cross(v1,v2)
    return np.array([v_ans[0], v_ans[1], v_ans[2], r1*r2 - np.dot(v1,v2)])

def rotate_vector(vec, quat):
    quat_inv = (-quat[0],-quat[1],-quat[2],quat[3])
    vec_as_quat = (vec[0], vec[1], vec[2], 0.)
    qP = quaternion_multiplication(quat, vec_as_quat)
    result = quaternion_multiplication(qP, quat_inv)
    return result[:3]




def open_saved_scene(scene_file, test_dir, shapes_list, motion_script, mobile_object_IDs, mobile_object_types, held_fixed_list):
    scene_data = file_handling.read_csv_file(scene_file, [str, float, float, float, float, float, float, float, float, float, float, int])

    # load plane
    planeID, plane_shapes_entry = load_object("plane", test_dir, useFixedBase=True)
    shapes_list.append(plane_shapes_entry)
    motion_script.append([])
    add_to_motion_script(planeID, 0., motion_script)

    binID = None

    for object_type,com_x,com_y,com_z,x,y,z,orient_x,orient_y,orient_z,orient_w,held_fixed in scene_data:
        if object_type=="bin":
            motion_script.append([])
            bin_scale = (0.75, 0.75, 0.75)
            bin_collision_ID = p.createCollisionShape(p.GEOM_MESH, meshScale=bin_scale, fileName=os.path.join("object models", "bin", "bin.obj"))
            bin_visual_shapeID = p.createVisualShape(p.GEOM_MESH, meshScale=bin_scale, fileName=os.path.join("object models", "bin", "bin.obj"))
            bin_mass = 1.-held_fixed
            binID = p.createMultiBody(bin_mass, bin_collision_ID, bin_visual_shapeID, (x, y, z * bin_scale[2]), (orient_x,orient_y,orient_z,orient_w))
            add_to_motion_script(binID, 0., motion_script)
            shapes_list.append(["bin", [0.0, 0.0, 0.0]])
        else:
            held_fixed_bool = bool(held_fixed)
            objectID, object_shapes_entry = load_object(object_type, test_dir, (com_x, com_y, com_z), useFixedBase=held_fixed_bool)
            shapes_list.append(object_shapes_entry)
            motion_script.append([])
            mobile_object_IDs.append(objectID)
            mobile_object_types.append(object_type)
            held_fixed_list.append(held_fixed_bool)
            p.resetBasePositionAndOrientation(objectID, (x, y, z), (orient_x,orient_y,orient_z,orient_w))

    return binID


def open_saved_scene_existing_objects(scene_file,mobile_object_IDs):
    scene_data = file_handling.read_csv_file(scene_file, [str, float, float, float, float, float, float, float, float, float, float, int])

    mobile_object_count = 0
    for object_type,com_x,com_y,com_z,x,y,z,orient_x,orient_y,orient_z,orient_w,held_fixed in scene_data:
        if object_type=="bin":
            pass #assume bin has already been made and is immovable
        else:
            objectID = mobile_object_IDs[mobile_object_count]
            p.resetBasePositionAndOrientation(objectID, (x, y, z), (orient_x,orient_y,orient_z,orient_w))
            p.resetBaseVelocity(objectID,(0.,0.,0.), (0.,0.,0.))
            mobile_object_count+=1


def save_scene(scene_file, binID, mobile_object_IDs, mobile_object_types, held_fixed_list):
    bin_pose, bin_orientation = p.getBasePositionAndOrientation(binID)
    bin_mass = p.getDynamicsInfo(binID,-1)[0]
    data = [["bin", 0., 0., 0., bin_pose[0], bin_pose[1], bin_pose[2]/0.75, bin_orientation[0], bin_orientation[1], bin_orientation[2], bin_orientation[3], int(1.-bin_mass)]]
    for i in np.arange(len(mobile_object_IDs)):
        ID = mobile_object_IDs[i]
        object_type = mobile_object_types[i]
        held_fixed = held_fixed_list[i]
        pose,orientation = p.getBasePositionAndOrientation(ID)
        COM = p.getDynamicsInfo(ID,-1)[3]
        data.append([object_type, COM[0], COM[1], COM[2], pose[0], pose[1], pose[2], orientation[0], orientation[1], orientation[2], orientation[3], int(held_fixed)])
    file_handling.write_csv_file(scene_file, "object_type,COM_x,COM_y,COM_z,x,y,z,orient_x,orient_y,orient_z,orient_w,held_fixed", data)


def save_scene_with_shifted_COMs(original_scene_file, new_scene_file, new_COM_list):
    original_scene_data = file_handling.read_csv_file(original_scene_file, [str, float, float, float, float, float, float, float, float, float, float, int])
    new_scene_data = []

    object_count=0
    for object_type,com_x,com_y,com_z,x,y,z,orient_x,orient_y,orient_z,orient_w,held_fixed in original_scene_data:
        new_COM = new_COM_list[object_count]
        unrotated = [-com_x+new_COM[0], -com_y+new_COM[1], -com_z+new_COM[2]]
        orientation = (orient_x,orient_y,orient_z,orient_w)
        if object_type != "bin":
            rotated = rotate_vector(unrotated, orientation)
        else:
            rotated = unrotated
        new_scene_data.append([object_type,new_COM[0],new_COM[1],new_COM[2],x+rotated[0],y+rotated[1],z+rotated[2],orient_x,orient_y,orient_z,orient_w,held_fixed])
        object_count+=1
    file_handling.write_csv_file(new_scene_file, "object_type,COM_x,COM_y,COM_z,x,y,z,orient_x,orient_y,orient_z,orient_w,held_fixed", new_scene_data)




def add_to_motion_script(id, time_val, motion_script):
    position, orientation = p.getBasePositionAndOrientation(id)
    motion_script[id].append([time_val,position[0],position[1],position[2],orientation[0],orientation[1],orientation[2],orientation[3]])


def PLY_header_str(num_points):
    return "ply\n"\
              + "format ascii 1.0\n"\
              + "comment point cloud generated from RGBD image taken in PyBullet simulation\n"\
              + "element vertex " + str(num_points)+"\n"\
              + "property float x\n"\
              + "property float y\n"\
              + "property float z\n"\
              + "end_header"


def PLY_header_str_extended(num_points):
    return "ply\n"\
              + "format ascii 1.0\n"\
              + "comment point cloud generated from RGBD image taken in PyBullet simulation\n"\
              + "element vertex " + str(num_points)+"\n"\
              + "property float x\n"\
              + "property float y\n"\
              + "property float z\n"\
              + "property float red\n"\
              + "property float green\n"\
              + "property float blue\n"\
              + "property float nx\n"\
              + "property float ny\n"\
              + "property float nz\n"\
              + "end_header"


def write_PLY_files(dest_dir, view_matrix, proj_matrix, mobile_object_IDs):
    w, h, RGBA, depth, segmentation_mask = p.getCameraImage(640, 480, view_matrix, proj_matrix, renderer=p.ER_TINY_RENDERER)
    segmentation_numpy = np.array(segmentation_mask).reshape((h*w))
    #RGBA_numpy = np.array(RGBA).reshape((h, w, 4))
    depth_numpy = np.array(depth).reshape((h,w))

    '''img_numpy = np.array(RGBA).reshape((h, w, 4))
    img = Image.fromarray(img_numpy, "RGBA")
    image_filename = os.path.join(dest_dir, str(0).zfill(4) + ".png")
    img.save(image_filename)'''

    #using same hard-coded values as those in set_up_camera to create proj matrix
    near = 0.001
    far = 100.0

    #get matrices to transform pixels to 3D coordinates
    #based on https://stackoverflow.com/questions/69803623/how-to-project-pybullet-simulation-coordinates-to-rendered-frame-pixel-coordinat
    pm = np.array(proj_matrix).reshape((4,4))
    vm = np.array(view_matrix).reshape((4,4))
    tr = -vm.T[:3,3]
    R = vm.T[:3,:3]
    fm = pm.T#np.matmul(pm, vm.T)
    fm_inv = np.linalg.inv(fm)

    # based on https://gist.github.com/Shreeyak/9a4948891541cb32b501d058db227fff
    pixel_x, pixel_y = np.meshgrid(np.linspace(-w/2, w/2 - 1, w), np.linspace(-h/2, h/2 - 1, h))
    points = np.array([pixel_x, pixel_y]).transpose(1,2,0)#.reshape(-1,2)
    points = np.append(points, np.zeros((h,w,1)), 2)
    points = np.append(points, np.ones((h,w,1)), 2)

    #get the depth information
    #based on https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=13370
    depth_numpy_corrected = far * near / (far - (far - near) * depth_numpy)
    depth_plane_multiplier = np.array([pm[0][0],pm[1][1],1.])

    corrected_points = np.zeros((h*w,3))
    point_index=0
    #this works for a camera with roll=0 and with either nonzero pitch or nonzero yaw, but not both. It should be good enough for experiments.
    #make it go faster. Current mission is to get a scene generation to run in way less than a second.
    for i in np.arange(h):
        for j in np.arange(w):
            #get points
            result = np.matmul(fm_inv, points[i][j])
            result = (result/result[3])[:3]
            result = np.multiply(result, depth_numpy_corrected[i][j]*depth_plane_multiplier)
            result[2] = -depth_numpy_corrected[i][j]
            #set points to world coordinates
            result += tr
            result = np.matmul(R, result)
            result[1]*=-1 #reflect across x axis
            corrected_points[point_index] = result
            point_index+=1
            #print(result)

    indices_without_surface = np.where(segmentation_numpy != 0)
    corrected_points = corrected_points[indices_without_surface]
    segmentation_numpy = segmentation_numpy[indices_without_surface]

    #kernel = np.ones((3, 3), np.uint8) #to dilate images to find contour normals

    for id in mobile_object_IDs:
        #based on https://gist.github.com/Shreeyak/9a4948891541cb32b501d058db227fff
        indicies_to_use = np.where(segmentation_numpy == id)
        corrected_points_to_use = corrected_points[indicies_to_use]

        if len(corrected_points_to_use) > 0:
            np.savetxt(os.path.join(dest_dir,"objID_"+str(id)+".ply"), corrected_points_to_use, fmt='%f',
                       header=PLY_header_str(len(corrected_points_to_use)), comments='', encoding='utf-8')

        '''indicies_obstacles = np.where(segmentation_numpy != id)
        corrected_points_obstacles = corrected_points[indicies_obstacles]

        # output to ply file
        np.savetxt(os.path.join(dest_dir, "objID_"+str(id)+"_obstacles.ply"), corrected_points_obstacles, fmt='%f',
                   header=PLY_header_str(len(corrected_points_obstacles)), comments='', encoding='utf-8')'''

def get_points_from_ply_file(points_file_path):
    header_size = 37
    #note: the final row is not a point and should be discarded
    points = np.loadtxt(points_file_path,delimiter=" ",skiprows=header_size, usecols=range(9))
    return points