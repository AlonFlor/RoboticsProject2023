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

#make images directory
imgs_dir = os.path.join(test_dir,"imgs")
os.mkdir(imgs_dir)

#physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

shapes_list = []
motion_script = []

object_IDs = []



# load plane
planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)
shapes_list.append(plane_shapes_entry)
motion_script.append([])
p_utils.add_to_motion_script(planeID, 0., motion_script)



#create objects
num_coms_to_sample = 4
startPos = (0.0, 0.0, 0.05)
startOrientation = p.getQuaternionFromEuler([0, -np.pi / 2, 0])

model_COMs = []
for i in np.arange(num_coms_to_sample):
    model_COMs.append((0., 0., 0.04+0.04*i))

for i in np.arange(num_coms_to_sample):
    # load object
    objectID, object_shapes_entry = p_utils.load_object("cracker_box", test_dir, model_COMs[i])
    print(i)
    object_shapes_entry[0] += "_"+str(i)
    shapes_list.append(object_shapes_entry)
    motion_script.append([])
    object_IDs.append(objectID)
    p.changeVisualShape(objectID,-1,rgbaColor=(1.,1.,1.,0.5))
    p.resetBasePositionAndOrientation(objectID, (startPos[0], startPos[1], startPos[2]+0.2*i), startOrientation)

#disable collision between the objects
for i in np.arange(1,num_coms_to_sample+1):
    for j in np.arange(1,i):
        p.setCollisionFilterPair(i,j,-1,-1,0)

#set the objects to be on top of each other
for i in np.arange(num_coms_to_sample):
    p.resetBasePositionAndOrientation(i+1, (startPos[0]+0.08-model_COMs[i][2], startPos[1], startPos[2]), startOrientation)


#create pusher
pusher_radius = 0.01
pusher_height = 0.1
pusher_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=pusher_radius, height=pusher_height)
pusher_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=pusher_radius, length=pusher_height)
pusherID = p.createMultiBody(1., pusher_shapeID, pusher_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))
motion_script.append([])
shapes_list.append(["pusher",[0.0,0.0,0.0]])
object_IDs.append(pusherID)




fps = 24.


view_matrix, proj_matrix = p_utils.set_up_camera(startPos, 0.5)

image_num = 0
p.resetBasePositionAndOrientation(pusherID, (startPos[0], startPos[1]-0.15, startPos[2]+0.05), (0., 0., 0., 1.))
pusher_end = np.array([startPos[0], startPos[1]+0.1, startPos[2]+0.05])
image_num = p_utils.push(pusher_end, pusherID, dt, object_IDs, fps, view_matrix, proj_matrix, imgs_dir, image_num, motion_script)




p.disconnect()


#write motion script and shapes file
print(shapes_list)
file_handling.write_records_and_motion_script(shapes_list, test_dir, motion_script)

#make a video from saved images
p_utils.make_video(test_dir,imgs_dir)
