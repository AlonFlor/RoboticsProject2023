import random
import numpy as np
import pybullet as p
import pybullet_utilities as p_utils
import file_handling
import os

# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)
os.mkdir(test_dir)

#physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.


view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)

# load plane
planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)


mobile_object_types = []
mobile_object_IDs = []


#load bin walls
bin_scale = (0.5,0.5,0.5)
bin_collision_ID = p.createCollisionShape(p.GEOM_MESH, meshScale = bin_scale, fileName=os.path.join("object models","bin","bin.obj"))
bin_visual_shapeID = p.createVisualShape(p.GEOM_MESH, meshScale = bin_scale, fileName=os.path.join("object models","bin","bin.obj"))
binID = p.createMultiBody(0., bin_collision_ID, bin_visual_shapeID, (0., 0., 0.4*bin_scale[2]), p.getQuaternionFromEuler([-np.pi / 2, 0., 0.]))


#define objects
available_objects = ["cracker_box", "pudding_box", "master_chef_can", "hammer", "tomato_soup_can", "wrench", "mustard_bottle"]
object_COMs = [(-0.01,-0.01,0.08), (0.0,0.0,0.015), (-0.015,-0.01,0.06), (-0.06,0.06,0.015), (-0.01,0.085,0.04), (0.01,-0.04,0.004), (-0.015,-0.023,0.07)]
number_of_each_object = [4, 1, 3, 0, 3, 0,2]
number_of_objects = 0
for number_of_object in number_of_each_object:
    number_of_objects += number_of_object


def add_an_object(i, xy_range, z_range, random_loc_or=True):
    #update the availability of the chosen object and the number of objects in total
    number_of_each_object[i] -= 1
    global number_of_objects
    number_of_objects = 0
    for number_of_object in number_of_each_object:
        number_of_objects += number_of_object

    if random_loc_or:
        #choose a random location and orientation for the new object
        location = (random.uniform(xy_range[0], xy_range[1]), random.uniform(xy_range[0], xy_range[1]), random.uniform(z_range[0], z_range[1]))
        orientation = np.array([random.uniform(0.,1.), random.uniform(0.,1.), random.uniform(0.,1.), random.uniform(0.,1.)])
        orientation = tuple(orientation / np.linalg.norm(orientation))

    #add the new object
    objectID, _ = p_utils.load_object(available_objects[i], test_dir, object_COMs[i])
    mobile_object_IDs.append(objectID)
    mobile_object_types.append(available_objects[i])
    if random_loc_or:
        p.resetBasePositionAndOrientation(objectID, location, orientation)


#put the target object first so it is at the bottom of the pile
add_an_object(1, (-0.05,0.05), (0.15,0.25), random_loc_or=False)

#add all other objects
while number_of_objects>0:
    #choose the next object. If it is not available, choose another one
    i = random.randint(0, len(available_objects) - 1)
    while number_of_each_object[i] == 0:
        i = random.randint(0,len(available_objects)-1)

    add_an_object(i, (-0.1,0.1), (0.2,0.4))

    #start the objects' motion and let the objects settle
    for step_num in np.arange(2./dt):
        p.stepSimulation()

#save the scene
held_fixed_list = [False for item in mobile_object_types]
p_utils.save_scene(os.path.join(test_dir,"saved_scene.csv"), binID, mobile_object_IDs, mobile_object_types, held_fixed_list)

#print an image of the scene
p_utils.print_image(view_matrix, proj_matrix, test_dir, 0)


p.disconnect()
