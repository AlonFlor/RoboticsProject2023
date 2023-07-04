import pybullet as p
import pybullet_utilities as p_utils
import os
import numpy as np
import file_handling

#physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)

# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)
os.mkdir(test_dir)



mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []


planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)

def create_obj(object_type, com, position, orientation):
    objectID, _ = p_utils.load_object(object_type, test_dir, com)
    mobile_object_IDs.append(objectID)
    mobile_object_types.append(object_type)
    p.resetBasePositionAndOrientation(objectID, position, orientation)


point_x_range = (-0.2,0.2)
point_y_range = (-0.2,0.2)
point_z_range = (0.2, 0.3)

'''#create a cracker box object with the COM that I prefer
position = p_utils.generate_point(point_x_range,point_y_range,point_z_range)
orientation = p.getQuaternionFromEuler((0.,np.pi/2, p_utils.generate_num((0.,2.*np.pi))))
create_obj("cracker_box", (-0.01,-0.01,0.08), position, orientation)'''


#find the acceptable COM bounds
#com_x_range, com_y_range, com_z_range = p_utils.get_COM_bounds("cracker_box")
object_type_com_bounds_and_test_points = {}
object_type_com_bounds_and_test_points["cracker_box"] = p_utils.get_com_bounds_and_test_points_for_object_type("cracker_box", 0.7, 0.8, 0.7)
object_type_com_bounds_and_test_points["master_chef_can"] = p_utils.get_com_bounds_and_test_points_for_object_type("master_chef_can", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["adjustable_wrench"] = p_utils.get_com_bounds_and_test_points_for_object_type("adjustable_wrench", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["pudding_box"] = p_utils.get_com_bounds_and_test_points_for_object_type("pudding_box", 0.7, 0.7, 0.7)
object_type_com_bounds_and_test_points["mustard_bottle"] = p_utils.get_com_bounds_and_test_points_for_object_type("mustard_bottle", 0.7, 0.7, 0.7)

'''#generate more cracker box objects with different COMs
for i in range(3):
    position = p_utils.generate_point(point_x_range, point_y_range, point_z_range)
    orientation = p.getQuaternionFromEuler((0., np.pi / 2, p_utils.generate_num((0., 2. * np.pi))))
    new_com = p_utils.generate_point(com_x_range, com_y_range, com_z_range)
    print("new com:",new_com)

    create_obj("cracker_box", new_com, position, orientation)'''


available_objects = ["cracker_box", "pudding_box", "master_chef_can", "adjustable_wrench", "mustard_bottle"]
number_of_each_object = [1, 1, 1, 1, 0]
object_COMs = [(-0.01,-0.01,0.08), (0.0,0.0,0.015), (-0.015,-0.01,0.06),  (0.01,-0.05,0.005), (-0.015,-0.023,0.07)]
#generate objects with different COMs
for i in range(len(available_objects)):
    for j in range(number_of_each_object[i]):
        position = p_utils.generate_point(point_x_range, point_y_range, point_z_range)
        orientation = p.getQuaternionFromEuler((0., np.pi / 2, p_utils.generate_num((0., 2. * np.pi))))
        if available_objects[i]=="master_chef_can":
            orientation = p.getQuaternionFromEuler((0.,0.,0.))
        #com_x_range, com_y_range, com_z_range = object_type_com_bounds_and_test_points[available_objects[i]]["com_bounds"]
        #new_com = p_utils.generate_point(com_x_range, com_y_range, com_z_range)
        #print("new com:",new_com)

        #create_obj(available_objects[i], new_com, position, orientation)
        create_obj(available_objects[i], object_COMs[i], position, orientation)


#simulate the scene, let it settle
import time

time_amount = 5.
count=0
while time_amount > 0:
    time_val = count * dt
    count += 1

    p.stepSimulation()

    time.sleep(dt)
    time_amount -= dt

#save the scene
held_fixed_list = [False for item in mobile_object_types]
p_utils.save_scene_no_bin(os.path.join(test_dir,"saved_scene.csv"), mobile_object_IDs, mobile_object_types, held_fixed_list)

#print an image of the scene
p_utils.print_image(view_matrix, proj_matrix, test_dir, None, "scene_img")


p.disconnect()

