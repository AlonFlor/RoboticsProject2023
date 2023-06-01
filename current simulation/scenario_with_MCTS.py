import numpy as np
import pybullet as p
import file_handling
import pybullet_utilities as p_utils
import MCTS_v1
import os


physicsClient = p.connect(p.DIRECT)
#physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

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
    binID = p_utils.open_saved_scene(scene_file, action_test_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list)

    action_type, point_1, point_2 = action_to_get_here

    if action_type == "grasp":
        pass
    else:
        # create cylinder
        cylinderID = p_utils.create_cylinder(MCTS_v1.pushing_point_free_space_radius, 0.05)
        p.resetBasePositionAndOrientation(cylinderID, point_1+MCTS_v1.cylinder_height_offset, (0., 0., 0., 1.))

        #push
        push_vector = point_2 - point_1
        push_vector = MCTS_v1.push_distance * push_vector / np.linalg.norm(push_vector)
        pusher_end = point_1 + push_vector + MCTS_v1.cylinder_height_offset
        available_image_num = p_utils.push(pusher_end, cylinderID, dt, mobile_object_IDs, 24, view_matrix, proj_matrix, imgs_dir, available_image_num, None, time_out=2.)
        available_image_num = p_utils.let_time_pass(cylinderID, dt, mobile_object_IDs, 24, view_matrix, proj_matrix, imgs_dir, available_image_num, None)
        p.removeBody(cylinderID)

        p_utils.save_scene(os.path.join(action_test_dir, "scene.csv"), binID, mobile_object_IDs, mobile_object_types,held_fixed_list)
    return available_image_num

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 0, -75)


def run_scenario(scene_number, accurate_COMs, target_index, with_MCTS_images=False):
    # make directory for simulation files
    testNum = 1
    while os.path.exists("test" + str(testNum)):
        testNum += 1
    test_dir = "test" + str(testNum)
    os.mkdir(test_dir)

    # image stuff
    scenario_image_num = 0
    image_folder = os.path.join(test_dir, "imgs")
    os.mkdir(image_folder)

    previous_scene_file = os.path.join("scenes",f"scene_{scene_number}.csv")
    if not accurate_COMs:
        scene_data = file_handling.read_csv_file(previous_scene_file, [str, float, float, float, float, float, float, float, float, float, float, int])
        COM_list_per_object = {"bin":(0.,0.,0.), "cracker_box":(-0.02,-0.04,0.16), "pudding_box":(0.03,0.02,0.015), "master_chef_can":(0.02,-0.02,0.06)}
        new_COM_list = []
        for object_type,com_x,com_y,com_z,x,y,z,orient_x,orient_y,orient_z,orient_w,held_fixed in scene_data:
            new_COM_list.append(COM_list_per_object[object_type])


    scenario_loop_index = 0
    while True:
        #get the scene
        scene_file = os.path.join(test_dir,f"scene_{scenario_loop_index}.csv")
        file_handling.copy_file(previous_scene_file, scene_file)
        if not accurate_COMs:
            scene_file_for_MCTS = os.path.join(test_dir, f"scene_{scenario_loop_index}_shifted_COM.csv")
            p_utils.save_scene_with_shifted_COMs(scene_file, scene_file_for_MCTS, new_COM_list)

        #apply MCTS
        MCTS_dir = os.path.join(test_dir,f"MCTS_{scenario_loop_index}")
        os.mkdir(MCTS_dir)
        if accurate_COMs:
            if with_MCTS_images:
                next_action = MCTS_v1.MCTS(MCTS_dir, dt, scene_file, target_index, view_matrix, proj_matrix)
            else:
                next_action = MCTS_v1.MCTS(MCTS_dir, dt, scene_file, target_index)
        else:
            if with_MCTS_images:
                next_action = MCTS_v1.MCTS(MCTS_dir, dt, scene_file_for_MCTS, target_index, view_matrix, proj_matrix)
            else:
                next_action = MCTS_v1.MCTS(MCTS_dir, dt, scene_file_for_MCTS, target_index)
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)

        #apply the action found by MCTS
        print("applying action",scenario_loop_index,"\t",next_action)
        action_dir = os.path.join(test_dir,f"action_{scenario_loop_index}")
        os.mkdir(action_dir)
        if next_action[0]=="grasp":
            mobile_object_IDs = []
            mobile_object_types = []
            held_fixed_list = []
            binID = p_utils.open_saved_scene(scene_file, action_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list)
            point_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=MCTS_v1.pushing_point_free_space_radius)
            p.createMultiBody(baseCollisionShapeIndex=point_collision_shape, basePosition=next_action[1])
            p.createMultiBody(baseCollisionShapeIndex=point_collision_shape, basePosition=next_action[2])
            p_utils.print_image(view_matrix,proj_matrix,test_dir,0,"_final result")
            scenario_loop_index += 1
            break
        scenario_image_num = apply_action_in_scenario(scene_file, action_dir, next_action, image_folder, scenario_image_num)
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)

        #save the location of the scene for the next iteration of the loop.
        previous_scene_file = os.path.join(action_dir,"scene.csv")

        scenario_loop_index += 1

    p_utils.make_video(test_dir,image_folder)
    print(f"Took {scenario_loop_index} attempts")

    return scenario_loop_index

number_of_tries = 5
tally_for_accurate_COMs = 0
tally_for_inaccurate_COMs = 0
for i in np.arange(number_of_tries):
    tally_for_accurate_COMs += run_scenario(9, True, 1)
for i in np.arange(number_of_tries):
    tally_for_inaccurate_COMs += run_scenario(9, False, 1)

print(f"With accurate COMs, total number of moves across {number_of_tries} trials was {tally_for_accurate_COMs}")
print(f"With the wrong COMs, total number of moves across {number_of_tries} trials was {tally_for_inaccurate_COMs}")

#run_scenario(3,True,8,True)

p.disconnect()
