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

#load object
objectID, object_shapes_entry = p_utils.load_object("cracker_box", test_dir, (0.,0.,0.04))
shapes_list.append(object_shapes_entry)
motion_script.append([])
object_IDs.append(objectID)


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

view_matrix, proj_matrix = p_utils.set_up_camera(p.getBasePositionAndOrientation(objectID)[0], 0.5)
startPos = (0.0, 0., 0.05)
startOrientation = p.getQuaternionFromEuler([0, -np.pi / 2, 0])

image_num = 0

for i in np.arange(-0.2,0.1,0.04):

    pusher_x_offset = i
    #print(i,"\t\t",startPos[0]+pusher_x_offset)
    p.resetBasePositionAndOrientation(pusherID, (startPos[0]+pusher_x_offset, startPos[1]-0.15, startPos[2]+0.05), (0., 0., 0., 1.))
    p.resetBasePositionAndOrientation(objectID, startPos, startOrientation)
    pusher_end = np.array([startPos[0]+pusher_x_offset, startPos[1]+0.1, startPos[2]+0.05])
    image_num = p_utils.push(pusher_end, pusherID, object_IDs, dt, fps, view_matrix, proj_matrix, imgs_dir, image_num, motion_script)



p.disconnect()


#write motion script and shapes file
file_handling.write_records_and_motion_script(shapes_list, test_dir, motion_script)

#make a video from saved images
p_utils.make_video(test_dir,imgs_dir)
