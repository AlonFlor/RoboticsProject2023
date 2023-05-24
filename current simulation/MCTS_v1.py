import random
import time

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

physicsClient = p.connect(p.DIRECT)
#physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 0, -75)

push_distance = 0.1
reward_discount = 0.9

#pushing_points_per_object = 40.
pushing_point_spacing = 0.04
pushing_point_free_space_radius = 0.015 / 2
pushing_point_displacement = 2.1*pushing_point_free_space_radius
explore_factor = 0.5

maximum_depth = 5


class node:
    def __init__(self,parent, action=None):
        self.parent = parent
        self.action_to_get_here = action
        self.node_reward = 0.
        self.sum_of_rewards = 0.
        self.number_of_visits = 0
        self.children = []
        self.unrecognized_children = []
        self.officially_unexplored_actions = []
        if parent is not None:
            self.depth = parent.depth + 1
        else:
            self.depth = 0

        # make directory for simulation files
        self.nodeNum = 0
        node_dir_path = os.path.join(test_dir,"node_" + str(self.nodeNum))
        while os.path.exists(node_dir_path):
            self.nodeNum += 1
            node_dir_path = os.path.join(test_dir,"node_" + str(self.nodeNum))
        self.node_dir = node_dir_path
        os.mkdir(self.node_dir)

    def create_child(self):
        #get the action
        action = self.officially_unexplored_actions.pop(0)
        for index, child in self.unrecognized_children:
            index -= 1

        #see if the child was already created in simulation. If it was, recognize it as a child.
        count = 0
        for index, child in self.unrecognized_children:
            if index==0:
                self.children.append(child)
                self.unrecognized_children.pop(count)
                return
            count +=1
        #If the child was not created in simulation, then create it.
        self.children.append(node(self, action))

    def create_or_open_unrecognized_child(self):
        #This function is for simulation rollouts. Create a child if it does not exist. Return its index if it did already exist, -1 otherwise.
        index = random.randrange(len(self.officially_unexplored_actions))   #randomly select an action

        unofficially_explored_action_indices = [i for i,child in self.unrecognized_children]
        if index in unofficially_explored_action_indices:
            return self.unrecognized_children.index(index)

        action = self.officially_unexplored_actions[index]
        self.unrecognized_children.append([index, node(self, action)])
        return -1

    def apply_action(self):

        reward = 0.

        #steps to skip if this is the root node
        if self.action_to_get_here is not None:
            #open scene before
            binID = p_utils.open_saved_scene(os.path.join(self.parent.node_dir, "scene.csv"), self.node_dir, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

            self.action_type, self.point_1, self.point_2 = self.action_to_get_here
            #action_type is either "push" or "grasp"
            #if it is "push", then point_1 is the pusher starting point, and point_2-point_1 forms the pushing vector
            #if it is "grasp", then point_1 and point_2 are the points of the grasp.


            #create cylinder
            cylinder_radius = 0.01
            cylinder_height = 0.1
            cylinder_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=cylinder_radius, height=cylinder_height)
            cylinder_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=cylinder_radius, length=cylinder_height)
            cylinderID = p.createMultiBody(1., cylinder_shapeID, cylinder_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))

            p.resetBasePositionAndOrientation(cylinderID, self.point_1, (0., 0., 0., 1.))

            if self.action_type == "grasp":
                #TODO replace this with actual robot grasp
                '''#create second cylinder for grasping
                cylinder2ID = p.createMultiBody(1., cylinder_shapeID, cylinder_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))
    
                p.resetBasePositionAndOrientation(cylinder2ID, self.point_2, (0., 0., 0., 1.))
    
                #grasp
                p_utils.grasp()
    
                p.removeBody(cylinder2ID)'''
                reward = 100. #congratulations on being able to grasp!
                #TODO check for successful grasp before giving award

            else:
                #push
                #TODO replace this with actual robot push
                push_vector = self.point_2 - self.point_1
                push_vector = push_distance * push_vector / np.linalg.norm(push_vector)
                pusher_end = self.point_1 + push_vector
                cylinder_original_point, _ = p.getBasePositionAndOrientation(cylinderID)
                p_utils.push(pusher_end, cylinderID, dt, time_out=2.)
                cylinder_new_point, _ = p.getBasePositionAndOrientation(cylinderID)

                #TODO if we are not filtering points by free space when selecting actions, some pushes will be dead ends.
                #TODO If they are, the action that opened that node was impermissable, so it will be enforced that this node will have no child nodes.
                #TODO I think it is better to filter points by free space ahead of time, to prevent the waste of simulating these dead ends
                #TODO Note: if we are filtering points, a point that would fail for pushing might still work for grasping,
                #TODO    so make the filtering-for-pushing step happen after splitting the points into for-pushing and for-grasping.
                actual_push_distance = np.linalg.norm(np.array(cylinder_new_point) - np.array(cylinder_original_point))
                if actual_push_distance < push_distance:
                    self.node_reward = -10
                    #TODO mark this node a dead end please

            p.removeBody(cylinderID)

            #save scene after
            p_utils.save_scene(os.path.join(self.node_dir,"scene.csv"), binID, mobile_object_IDs, mobile_object_types, held_fixed_list)
        else:
            #open scene in root node
            p_utils.open_saved_scene(os.path.join(self.node_dir, "scene.csv"), self.node_dir, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

        #TODO: delete this
        target_pos, _ = p.getBasePositionAndOrientation(mobile_object_IDs[0])
        if np.linalg.norm(np.array(target_pos) - np.array([-0.1, 0., target_pos[2]])) < 0.03:
            reward+=100.
        p_utils.print_image(view_matrix,proj_matrix,self.node_dir,0)

        p_utils.write_PLY_files(self.node_dir, view_matrix, proj_matrix, mobile_object_IDs)
        self.officially_unexplored_actions = self.generate_action_list()
        p.resetSimulation()

        return reward

    def back_propagate(self, reward):
        #update sum of rewards and number of times visited, for this node and for its parent nodes up the tree to the root.
        self.sum_of_rewards += reward
        self.number_of_visits += 1
        if self.parent != None:
            self.parent.back_propagate(reward_discount * reward)


    def generate_action_list(self):
        if self.depth == maximum_depth:
            return []

        candidate_list = []

        for id in mobile_object_IDs:
            basic_id_ply_str = "objID_"+str(id)

            id_ply_str = os.path.join(self.node_dir, basic_id_ply_str+".ply")
            #id_ply_obstacles_str = os.path.join(self.node_dir, basic_id_ply_str+"_obstacles.ply")
            id_ply_with_normals_str = os.path.join(self.node_dir, basic_id_ply_str+"_with_normals.ply")
            #id_ply_obstacles_with_normals_str = os.path.join(self.node_dir, basic_id_ply_str+"_obstacles_with_normals.ply")

            pcl_get_normals_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclnormal.exe\" " + id_ply_str + " " + id_ply_with_normals_str
            print(pcl_get_normals_command)
            os.popen(pcl_get_normals_command)

            file_handling.wait_for_file(id_ply_with_normals_str)

            #select points

            '''pcl_get_normals_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclnormal.exe\" " + id_ply_obstacles_str + " " + id_ply_obstacles_with_normals_str
            print(pcl_get_normals_command)
            os.popen(pcl_get_normals_command)
            file_handling.wait_for_file(id_ply_obstacles_with_normals_str)

            id_ply_pushing_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_pushing_points.ply")
            pcl_pushing_points_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclpush.exe\" "\
                                         + id_ply_with_normals_str + " " + id_ply_obstacles_with_normals_str + " " + id_ply_pushing_points_str
            print(pcl_pushing_points_command)
            os.popen(pcl_pushing_points_command)
            file_handling.wait_for_file(id_ply_pushing_points_str)'''

            '''id_ply_downsampled_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_down_sampled_points.ply")
            pcl_down_sample_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclsample.exe\" "\
                                      + id_ply_with_normals_str + f" {pushing_point_spacing} " + id_ply_downsampled_points_str
            print(pcl_down_sample_command)
            os.popen(pcl_down_sample_command)
            file_handling.wait_for_file(id_ply_downsampled_points_str)'''

            points = p_utils.get_points_from_ply_file(id_ply_with_normals_str)[:-1]
            #points = p_utils.get_points_from_ply_file(id_ply_downsampled_points_str)[:-1]
            points2 = np.copy(points)

            #move points out
            for i in np.arange(len(points)):
                new_2D_normal = np.array([points[i][6], points[i][7], 0.])
                new_2D_normal /= np.linalg.norm(new_2D_normal)

                points[i][:3] += pushing_point_displacement * new_2D_normal
                points2[i][:3] -= pushing_point_displacement * new_2D_normal

                points[i][6:9] = new_2D_normal
                points2[i][6:9] = -new_2D_normal

            points_sum = np.concatenate([points,points2], axis=0)

            id_ply_shifted_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_shifted_points.ply")
            np.savetxt(id_ply_shifted_points_str, points_sum, fmt='%f', header=p_utils.PLY_header_str_extended(len(points_sum)), comments='', encoding='utf-8')


            id_ply_downsampled_points_str = os.path.join(self.node_dir, basic_id_ply_str + "_down_sampled_points.ply")
            pcl_down_sample_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclsample.exe\" " \
                                      + id_ply_shifted_points_str + f" {pushing_point_spacing} " + id_ply_downsampled_points_str
            print(pcl_down_sample_command)
            os.popen(pcl_down_sample_command)
            file_handling.wait_for_file(id_ply_downsampled_points_str)

            points = p_utils.get_points_from_ply_file(id_ply_downsampled_points_str)[:-1]
            for i in np.arange(len(points)):
                candidate_list.append((points[i][:3], points[i][:3] - points[i][6:9])) #second point is first point pushed away from normal.
                # TODO: make the second point the pair partner of the first point instead.

        point_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=pushing_point_free_space_radius)
        point_collision_IDs = []
        for i,point_and_normal in enumerate(candidate_list):
            point, normal = point_and_normal
            point_id = p.createMultiBody(baseCollisionShapeIndex=point_collision_shape, basePosition=point)
            point_collision_IDs.append(point_id)
        p.performCollisionDetection()

        filtered_candidate_list = []
        list_to_export = []
        for i,point_id in enumerate(point_collision_IDs):
            contact_results = p.getContactPoints(point_id)
            if len(contact_results)==0:
                filtered_candidate_list.append(("push", candidate_list[i][0], candidate_list[i][1]))
                list_to_export.append(candidate_list[i][0])
            else:
                p.removeBody(point_id)

        p_utils.print_image(view_matrix, proj_matrix, self.node_dir, 1)

        pushing_points_ply_path_str = os.path.join(self.node_dir, "pushing_points.ply")
        list_to_export = np.array(list_to_export)
        np.savetxt(pushing_points_ply_path_str, list_to_export, fmt='%f', header=p_utils.PLY_header_str(len(list_to_export)), comments='', encoding='utf-8')

        return filtered_candidate_list


        #TODO There should only be one cpp file that handles everything done here from when the first PLY file is printed until it is retrieved again.
        #TODO Have Eric change his code so that a single PLY file come in, with points labeled by object ID or an ID for the background, and out comes a PLY file with points and edges.
        #TODO each edge is two possible pushing actions (point_1, point_2) and (point_2, point_1)
        #TODO points not assigned to edges need a normal to generate their counterpart point for pushing.

    def select_node_to_expand(self, explore_factor):
        #If a node is childless, select that node. Unrecognized children have never been officially visited, so they do not count as children here.
        if len(self.children) == 0:
            return self
        #If a node has unexplored actions, select that node
        if len(self.officially_unexplored_actions) > 0:
            return self
        #upper confidence bound
        max_score = -1
        select_index = 0
        for i,child in enumerate(self.children):
            UCB_score = child.sum_of_rewards / child.number_of_visits + explore_factor*np.sqrt(2*np.log(self.number_of_visits) / child.number_of_visits)
            if UCB_score > max_score:
                max_score = UCB_score
                select_index = i
        return self.children[select_index].select_node_to_expand(explore_factor)

'''
Algorithm:
Create root node, then repeat:
    1. select a node by calling select_node_to_expand from the root node, it will be called recursively until a suitable node to expand is found.
        if the selected node has no actions and no children, exit the loop.
    2. generate a node and remove an unexplored action using create_child
    3. For the newly created node n:
        3a. run the action taken from its parent using apply_action, save the reward in a variable, save the unexplored actions in the node
        3b. Repeat:
            3ba. from the current node's unexplored actions, create an unrecognized child using create_or_open_unrecognized_child, and temporarily make that node the new current node
            if that child already exists:
                3bb. run the action taken from its parent using apply_action, and save the reward in a variable, save the unexplored actions in the node.
                    If that list is empty, end the loop.
            else:
                3bc. get the reward and unexplored actions of the existing unrecognized child node
        3c. take the reward saved from 3a, 3bb, or 3bc (there should only be one variable that gets overwritten whenever 3bb or 3bc are called), and set node_reward for n to that value
        3d. then update the sum of rewards and the number of times visited for node n and up its chain by calling back_propagate for node n
Return the action_to_get_here of the direct child of the root that has the highest reward.

To create the root node, call the constructor, then apply_action.



Parallelization policy:
Step 1 is run serially.
    The algorithm is changed to branch by returning multiple nodes each time, and keeping track of the number of unexplored actions (not number of nodes).
    Once the number of unexplored actions collected is equal to the number of processors, or there are no more unexplored actions to collect, then finish this step.
    If the number of actions collected is 0, exit the loop like before.
Step 2 is run serially, to avoid collisions/race conditions in numbering the node files.
    This step is done for each unexplored action in each node in the list of nodes returned by step one.
    It returns a list of newly created child nodes for steps 3a and 3b.
Step 3a is run in parallel, one process per node from the list from step 2.
Step 3ba is run serially, to avoid collisions/race conditions in numbering the node files.
Steps 3bb and 3bc (and the if-else statement enclosing them) are run in parallel, picking up on the same processes used for 3a.
3c is is run in parallel, picking up on the same processes used for 3a, 3bb, and 3bc.
3d is run serially, to avoid race conditions in updating rewards and number of times visited up the chain.
'''

root_node = node(None)
#load a scene to the node
file_handling.copy_file(os.path.join("scenes",f"scene_{9}_shifted_COM.csv"), os.path.join(root_node.node_dir, "scene.csv"))
root_node.apply_action()
#root_node.create_child()

while True:
    node_to_expand = root_node.select_node_to_expand(explore_factor)
    if len(node_to_expand.children)==0 and len(node_to_expand.officially_unexplored_actions)==0:
        break
    node_to_expand.create_child()
    n = node_to_expand.children[-1]
    reward = n.apply_action()
    current_node = n
    while len(current_node.officially_unexplored_actions) != 0:
        unrecognized_child_index = current_node.create_or_open_unrecognized_child()
        current_node = current_node.unrecognized_children[unrecognized_child_index][1]
        reward = current_node.apply_action()
    n.back_propagate(reward)

'''start = np.array([0.15,0.,0.02])
end = np.array([start[0]-0.25, start[1], start[2]])
node1 = node(root_node, ("push", start, end))
node1.apply_action()'''


'''For timing purposes. Copy where needed, perhaps to a single round of apply_action and generate_action_list:

start_time = time.perf_counter_ns()
print('Answer:', pbe.fibinacci_cpp(n))
print('Time:', (time.perf_counter_ns() - start_time) / 1e9, 's')
'''

p.disconnect()
