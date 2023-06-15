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

#create a cracker box object with the COM that I prefer
position = p_utils.generate_point(point_x_range,point_y_range,point_z_range)
orientation = p.getQuaternionFromEuler((0.,np.pi/2, p_utils.generate_num((0.,2.*np.pi))))
create_obj("cracker_box", (-0.01,-0.01,0.08), position, orientation)


#find the acceptable COM bounds
com_x_range, com_y_range, com_z_range = p_utils.get_COM_bounds("cracker_box")

#generate more cracker box objects with different COMs
for i in range(3):
    position = p_utils.generate_point(point_x_range, point_y_range, point_z_range)
    orientation = p.getQuaternionFromEuler((0., np.pi / 2, p_utils.generate_num((0., 2. * np.pi))))
    new_com = p_utils.generate_point(com_x_range, com_y_range, com_z_range)
    print("new com:",new_com)

    create_obj("cracker_box", new_com, position, orientation)


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
p_utils.print_image(view_matrix, proj_matrix, test_dir, 0)


p.disconnect()

