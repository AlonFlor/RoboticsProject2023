import pybullet as p
import numpy as np
import os
import make_URDF
from PIL import Image


def load_object(name, test_dir, model_COM=(0.,0.,0.), useFixedBase=False):
    i=1
    extra_designation = ""
    while os.path.exists(os.path.join(test_dir,name+extra_designation+".urdf")):
        extra_designation = "_" + str(i).zfill(3)
        i+=1
    make_URDF.write_URDF_from_mesh(name, test_dir, model_COM, extra_designation)
    objectID = p.loadURDF(os.path.join(test_dir,name+extra_designation+".urdf"), useFixedBase=useFixedBase)

    return objectID, [name,model_COM]



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




def push(pusher_end, pusherID, object_IDs, dt, fps, view_matrix, proj_matrix, imgs_dir, available_image_num, motion_script):
    count = 0
    image_num = available_image_num + 0

    pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
    while np.linalg.norm(np.array(pusher_position) - pusher_end) > 0.01:
        time_val = count * dt
        if (time_val * fps - int(time_val * fps) < 0.0001):
            for ID in object_IDs:
                add_to_motion_script(ID, time_val, motion_script)

            print_image(view_matrix, proj_matrix, imgs_dir, image_num)
            image_num+=1

        p.stepSimulation()

        # reset the pusher
        pusher_position = p.getBasePositionAndOrientation(pusherID)[0]
        pusher_position = (pusher_position[0], pusher_position[1], pusher_end[2])  # keep the height constant
        p.resetBasePositionAndOrientation(pusherID, pusher_position, (0., 0., 0., 1.))
        pusher_displacement_from_destination = pusher_end - np.array(pusher_position)
        pusher_dist_from_destination = np.linalg.norm(pusher_displacement_from_destination)
        pusher_speed = .15
        new_pusher_velocity = pusher_speed * pusher_displacement_from_destination / pusher_dist_from_destination
        p.resetBaseVelocity(pusherID, (new_pusher_velocity[0], new_pusher_velocity[1], new_pusher_velocity[2]), (0., 0., 0.))
        p.applyExternalForce(pusherID, -1, (0., 0., 9.8), (0., 0., 0.), p.LINK_FRAME)  # antigravity

        count += 1

    return image_num




def quaternion_multiplication(q1, q2):
    r1 = q1[3]
    r2 = q2[3]
    v1 = np.array([q1[0], q1[1], q1[2]])
    v2 = np.array([q2[0], q2[1], q2[2]])
    v_ans = r1*v2 + r2*v1 + np.cross(v1,v2)
    return (v_ans[0], v_ans[1], v_ans[2], r1*r2 - np.dot(v1,v2))






def add_to_motion_script(id, time_val, motion_script):
    position, orientation = p.getBasePositionAndOrientation(id)
    motion_script[id].append([time_val,position[0],position[1],position[2],orientation[0],orientation[1],orientation[2],orientation[3]])