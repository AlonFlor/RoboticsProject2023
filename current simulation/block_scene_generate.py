import pybullet as p
import numpy as np
import pybullet_utilities as p_utils
import os
import random

import make_URDF


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
view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 1.75, 45, -65)


mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []

object_scale = 0.015

planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)


#A scene of YCB files is a single file that lists the type, COM, and pose of the YCB file.
#The same thing will be here. Block shapes have their URDF files in the object_models folder.
#Mass and friction of each block can be decided and changed as needed in-simulation.

#However, over here the code still produces URDF files along with the scene file, in case I need new URDF files to place in the object_models folder.

def create_block_obj(name, object_type, position, orientation):
    objectID = p.loadURDF(os.path.join(test_dir,name+".urdf"), globalScaling=object_scale)
    mobile_object_IDs.append(objectID)
    mobile_object_types.append(object_type)
    p.resetBasePositionAndOrientation(objectID, position, orientation)


def add_a_block_object(i, point_x_range, point_y_range, point_z_range):
    #update the availability of the chosen object and the number of objects in total
    number_of_each_object[i] -= 1
    global number_of_objects
    number_of_objects = 0
    for number_of_object in number_of_each_object:
        number_of_objects += number_of_object

    #choose a random location and orientation for the new object
    position = p_utils.generate_point(point_x_range, point_y_range, point_z_range)
    orientation = p.getQuaternionFromEuler((0., 0., p_utils.generate_num((0., 2. * np.pi))))

    #add the new object
    object_name = available_objects[i]
    extra_designation=""
    extra_designation_num = 1
    while os.path.exists(os.path.join(test_dir,object_name+extra_designation+".urdf")):
        extra_designation = "_" + str(extra_designation_num).zfill(2)
        extra_designation_num += 1
    print(available_objects[i])
    obj_data, default_masses, default_frictions = p_utils.get_block_object_type_info(available_objects[i])
    new_masses = []
    for mass in default_masses:
        new_masses.append(mass/len(default_masses))
    make_URDF.write_URDF_with_external_mass_and_friction_info(object_name, extra_designation, obj_data, new_masses, default_frictions, test_dir)
    create_block_obj(object_name+extra_designation, available_objects[i], position, orientation)



#define objects
available_objects = ["spray_gun", "hammer", "wrench", "snack_uniform"]
number_of_each_object = [1, 1, 0, 2]
number_of_objects = 0
for number_of_object in number_of_each_object:
    number_of_objects += number_of_object


#define ranges
point_x_range = (-0.2,0.2)
point_y_range = (-0.2,0.2)
point_z_range = (object_scale,object_scale+0.3)



#add all objects
while number_of_objects>0:
    #choose the next object. If it is not available, choose another one
    i = random.randint(0, len(available_objects) - 1)
    while number_of_each_object[i] == 0:
        i = random.randint(0,len(available_objects)-1)

    add_a_block_object(i, point_x_range, point_y_range, point_z_range)

    #start the objects' motion and let the objects settle
    for step_num in np.arange(2./dt):
        p.stepSimulation()

#save the scene
held_fixed_list = [False for item in mobile_object_types]
p_utils.save_scene_blocks_no_bin(os.path.join(test_dir,"scene.csv"), mobile_object_IDs, mobile_object_types, held_fixed_list)

#print an image of the scene
p_utils.print_image(view_matrix, proj_matrix, test_dir, None, "scene")


p.disconnect()




