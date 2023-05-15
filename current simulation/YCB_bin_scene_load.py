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
mobile_object_types = []
held_fixed_list = []


p_utils.open_saved_scene(os.path.join("scenes","scene_4.csv"), test_dir, shapes_list, motion_script, mobile_object_IDs, mobile_object_types, held_fixed_list)

#create pusher
pusher_radius = 0.01
pusher_height = 0.1
pusher_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=pusher_radius, height=pusher_height)
pusher_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=pusher_radius, length=pusher_height)
pusherID = p.createMultiBody(1., pusher_shapeID, pusher_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))
motion_script.append([])
shapes_list.append(["pusher",[0.0,0.0,0.0]])



fps = 24.
view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 45, -75)

#pusher_start_pos = (0.,-0.25,0.02)
pusher_start_pos = (0.2,0.,0.02)

image_num = 0

p.resetBasePositionAndOrientation(pusherID, pusher_start_pos, (0., 0., 0., 1.))
#pusher_end = np.array([pusher_start_pos[0], pusher_start_pos[1]+0.4, pusher_start_pos[2]])
pusher_end = np.array([pusher_start_pos[0]-0.2, pusher_start_pos[1], pusher_start_pos[2]])
image_num = p_utils.push(pusher_end, pusherID, mobile_object_IDs, dt, fps, view_matrix, proj_matrix, imgs_dir, image_num, motion_script)



p.disconnect()


#write motion script and shapes file
file_handling.write_records_and_motion_script(shapes_list, test_dir, motion_script)

#make a video from saved images
p_utils.make_video(test_dir,imgs_dir)
