import os
import numpy as np
import draw_data
import pybullet as p
import pybullet_utilities as p_utils
import file_handling

# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)

#make images directory
os.mkdir(test_dir)
imgs_dir = os.path.join(test_dir,"imgs")
os.mkdir(imgs_dir)

#physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 0.01#1./240.


shapes_list = []
motion_script = []


#for images recording
image_num = 0





# load plane
planeID, plane_shapes_entry = p_utils.load_object("plane", test_dir, useFixedBase=True)
shapes_list.append(plane_shapes_entry)
motion_script.append([])
p_utils.add_to_motion_script(planeID, 0., motion_script)

#load object
objectID, object_shapes_entry = p_utils.load_object("cracker_box", test_dir, (0.,0.,0.04))
shapes_list.append(object_shapes_entry)
motion_script.append([])

#create pusher
pusher_radius = 0.01
pusher_height = 0.1
pusher_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=pusher_radius, height=pusher_height)
pusher_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=pusher_radius, length=pusher_height)
pusherID = p.createMultiBody(1., pusher_shapeID, pusher_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))
motion_script.append([])
shapes_list.append(["pusher",[0.0,0.0,0.0]])


#get robot pose data
robot_lab_data_folder = "robot_lab_data"
pose_data, forces_data, end_effector_data = file_handling.read_robot_lab_data(robot_lab_data_folder)


#get start index when robot end effector is at minimal point
starting_index = 0
min_z = 10.
for index,row in enumerate(end_effector_data):
    if row[3] < min_z:
        min_z = row[3]
        starting_index = index





#with start index available, and assyming that the end time is the last time in the recording, we have the data to plot the movement of the object.
startPos = (0.0, 0., 0.05)
startOrientation = p.getQuaternionFromEuler([0, -np.pi / 2, 0])

initial_position = (pose_data[0], pose_data[1], startPos[2])
new_position = (initial_position[0] + pose_data[2], initial_position[1] + pose_data[3], initial_position[2])
angle_change = pose_data[4]

final_orientation_raw = p.getQuaternionFromEuler((0.,0.,angle_change))
final_orientation = p_utils.quaternion_multiplication(final_orientation_raw, startOrientation)

positions = []
orientations = []
pusher_positions = []
denom = float(len(end_effector_data)-1 - starting_index)
for i in np.arange(starting_index,len(end_effector_data)):
    fraction = float(i - starting_index) / denom
    positions.append((new_position[0]*fraction + initial_position[0]*(1.-fraction), new_position[1]*fraction + initial_position[1]*(1.-fraction), initial_position[2]))
    orientations.append(p.getQuaternionSlerp(startOrientation, final_orientation, fraction))
    pusher_positions.append((end_effector_data[i][1], end_effector_data[i][2], end_effector_data[i][3]))


p.resetBasePositionAndOrientation(objectID, positions[0], orientations[0])
view_matrix, proj_matrix = p_utils.set_up_camera(p.getBasePositionAndOrientation(objectID)[0], 0.5)

fps = 20.
for i in np.arange(starting_index,len(end_effector_data)):
    time_val = (i-starting_index)*dt
    if(time_val * fps - int(time_val * fps) < 0.0001):
        p_utils.add_to_motion_script(objectID, time_val, motion_script)
        p_utils.add_to_motion_script(pusherID, time_val, motion_script)

        p_utils.print_image(view_matrix, proj_matrix, imgs_dir, image_num)
        image_num+=1
    
    p.resetBasePositionAndOrientation(pusherID, pusher_positions[i-starting_index], (0., 0., 0., 1.))
    p.resetBasePositionAndOrientation(objectID, positions[i-starting_index], orientations[i-starting_index])
    p.stepSimulation()


p.disconnect()

#write motion script and shapes file
file_handling.write_records_and_motion_script(shapes_list, test_dir, motion_script)

#make a video from saved images
p_utils.make_video(test_dir,imgs_dir)
