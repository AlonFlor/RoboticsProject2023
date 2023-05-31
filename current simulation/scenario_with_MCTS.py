import numpy as np
import pybullet as p
import file_handling
import pybullet_utilities as p_utils
import MCTS_v1
import os

# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)
os.mkdir(test_dir)

physicsClient = p.connect(p.DIRECT)
#physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

scenario_image_num = 0
image_folder = os.path.join(test_dir, "imgs")
os.mkdir(image_folder)

'''
Plan:

1. start with scene
2. apply MCTS to get next action
3. simulate next action, save the images and make a video
'''

def apply_action_in_scenario(scene_file, action_test_dir, action_to_get_here, imgs_dir, available_image_num):
    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []
    binID = p_utils.open_saved_scene(scene_file, action_test_dir, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

    action_type, point_1, point_2 = action_to_get_here

    if action_type == "grasp":
        pass
    else:
        # create cylinder
        cylinderID = p_utils.create_cylinder(MCTS_v1.pushing_point_free_space_radius, 0.05)
        p.resetBasePositionAndOrientation(cylinderID, point_1, (0., 0., 0., 1.))

        #push
        push_vector = point_2 - point_1
        push_vector = MCTS_v1.push_distance * push_vector / np.linalg.norm(push_vector)
        pusher_end = point_1 + push_vector
        cylinder_original_point, _ = p.getBasePositionAndOrientation(cylinderID)
        available_image_num = p_utils.push(pusher_end, cylinderID, dt, mobile_object_IDs, 24, view_matrix, proj_matrix, imgs_dir, available_image_num, [], time_out=2.)
        available_image_num = p_utils.let_time_pass(0.2, cylinderID, dt, mobile_object_IDs, 24, view_matrix, proj_matrix, imgs_dir, available_image_num, [])
        p.removeBody(cylinderID)

        p_utils.save_scene(os.path.join(action_test_dir, "scene.csv"), binID, mobile_object_IDs, mobile_object_types,held_fixed_list)
    return available_image_num

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 0, -75)

#get the scene
scenario_loop_index = 0
original_scene_file = os.path.join("scenes",f"scene_{9}_shifted_COM.csv")
scene_file = os.path.join(test_dir,f"scene_{scenario_loop_index}.csv")

#apply MCTS
file_handling.copy_file(original_scene_file, scene_file)
MCTS_dir = os.path.join(test_dir,f"MCTS {scenario_loop_index}")
os.mkdir(MCTS_dir)
next_action = MCTS_v1.MCTS(MCTS_dir, dt, scene_file)#, view_matrix, proj_matrix)

#apply the action found by MCTS
action_dir = os.path.join(test_dir,f"action {scenario_loop_index}")
scenario_image_num = apply_action_in_scenario(scene_file, action_dir, next_action, image_folder, scenario_image_num)
