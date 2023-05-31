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

mobile_object_IDs = []



# load plane
planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)
shapes_list.append(plane_shapes_entry)
motion_script.append([])
p_utils.add_to_motion_script(planeID, 0., motion_script)


turned_orientation = p.getQuaternionFromEuler([0., -np.pi / 2, 0.])


#load bin walls
motion_script.append([])
bin_scale = (0.75,0.75,0.75)
bin_collision_ID = p.createCollisionShape(p.GEOM_MESH, meshScale = bin_scale, fileName=os.path.join("object models","bin","bin.obj"))
bin_visual_shapeID = p.createVisualShape(p.GEOM_MESH, meshScale = bin_scale, fileName=os.path.join("object models","bin","bin.obj"))
binID = p.createMultiBody(0., bin_collision_ID, bin_visual_shapeID, (0., 0., 0.4*bin_scale[2]), p.getQuaternionFromEuler([-np.pi / 2, 0., 0.]))
p_utils.add_to_motion_script(binID, 0., motion_script)
shapes_list.append(["bin",[0.0,0.0,0.0]])



#load objects
objectID, object_shapes_entry = p_utils.load_object("cracker_box", test_dir, (-0.01,-0.01,0.08))
shapes_list.append(object_shapes_entry)
motion_script.append([])
mobile_object_IDs.append(objectID)
p.resetBasePositionAndOrientation(objectID, (0.15,0.05,0.05), turned_orientation)

objectID, object_shapes_entry = p_utils.load_object("hammer", test_dir, (-0.06,0.06,0.015))
shapes_list.append(object_shapes_entry)
motion_script.append([])
mobile_object_IDs.append(objectID)
p.resetBasePositionAndOrientation(objectID, (-0.05,0.05,0.05), (0.,0.,0.,1.))

objectID, object_shapes_entry = p_utils.load_object("master_chef_can", test_dir, (-0.015,-0.01,0.06))
shapes_list.append(object_shapes_entry)
motion_script.append([])
mobile_object_IDs.append(objectID)
p.resetBasePositionAndOrientation(objectID, (0.15,-0.1,0.05), (0.,0.,0.,1.))

objectID, object_shapes_entry = p_utils.load_object("pudding_box", test_dir, (0.,0.,0.015))
shapes_list.append(object_shapes_entry)
motion_script.append([])
mobile_object_IDs.append(objectID)
p.resetBasePositionAndOrientation(objectID, (-0.15,-0.05,0.05), (0.,0.,0.,1.))



#create pusher
pusher_radius = 0.01
pusher_height = 0.05
pusher_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=pusher_radius, height=pusher_height)
pusher_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=pusher_radius, length=pusher_height)
pusherID = p.createMultiBody(1., pusher_shapeID, pusher_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))
motion_script.append([])
shapes_list.append(["pusher",[0.0,0.0,0.0]])
mobile_object_IDs.append(pusherID)




fps = 24.

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -65)

#pusher_start_pos = (0.,-0.25,0.02)
pusher_start_pos = (0.25,0.,0.05)

image_num = 0

p.resetBasePositionAndOrientation(pusherID, pusher_start_pos, (0., 0., 0., 1.))
#pusher_end = np.array([pusher_start_pos[0], pusher_start_pos[1]+0.4, pusher_start_pos[2]])
pusher_end = np.array([pusher_start_pos[0]-0.35, pusher_start_pos[1], pusher_start_pos[2]])
image_num = p_utils.push(pusher_end, pusherID, dt, mobile_object_IDs, fps, view_matrix, proj_matrix, imgs_dir, image_num, motion_script)
image_num = p_utils.let_time_pass(pusherID, dt, mobile_object_IDs, fps, view_matrix, proj_matrix, imgs_dir, image_num, motion_script)


p.disconnect()


#write motion script and shapes file
file_handling.write_records_and_motion_script(shapes_list, test_dir, motion_script)

#make a video from saved images
p_utils.make_video(test_dir,imgs_dir)
