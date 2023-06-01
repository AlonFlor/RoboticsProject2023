import random
import time

import numpy as np
import pybullet as p
import pybullet_utilities as p_utils
import file_handling
import os

push_distance = 0.15
reward_discount = 0.8
basic_reward = 100.

explore_factor = 0.5
maximum_depth = 4
pushing_point_free_space_radius = 0.015 / 2
cylinder_height_offset = np.array([0.,0.,0.02])
iteration_limit = 150

image_num = 0


def precompute_pushing_points_and_point_pairs(object_type, object_com):
    #Points are a np array, edges are a list of tuples, where each tuple contains the indices of a pair pushing points.
    precomputed_points_file = os.path.join("object models",object_type,"precomputed_pushing_points.csv")
    precomputed_pushing_point_pairs = file_handling.read_csv_file(precomputed_points_file, [float, float, float, float, float, float])
    precomputed_pushing_points = []
    edges = []
    for i in np.arange(len(precomputed_pushing_point_pairs)):
        precomputed_pushing_points.append(precomputed_pushing_point_pairs[i][:3])
        precomputed_pushing_points.append(precomputed_pushing_point_pairs[i][3:6])
        current_index = len(precomputed_pushing_points) -1
        edges.append((current_index, current_index-1))
    precomputed_pushing_points = np.array(precomputed_pushing_points)
    precomputed_pushing_points -= object_com
    return precomputed_pushing_points,edges


class node:
    def __init__(self,parent, test_dir, action=None):
        self.test_dir = test_dir
        self.parent = parent
        self.action_to_get_here = action
        self.node_reward = None
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
        self.children.append(node(self, self.test_dir, action))


    '''def get_winning_move_if_available(self, mobile_object_IDs, target_index):
        for i,action in enumerate(self.officially_unexplored_actions):
            if action[0] == "grasp":
                grasp_ray = p.rayTest(action[1], action[2])
                if len(grasp_ray) > 0:
                    grasped_object_id = grasp_ray[0][0]
                    if grasped_object_id == mobile_object_IDs[target_index]:
                        return i
        return -1'''

    def create_or_open_unrecognized_child(self, index=None):
        '''This function is for simulation rollouts. Create a child if it does not exist. Return its index if it did already exist, -1 otherwise.'''
        if index is None:
            index = random.randrange(len(self.officially_unexplored_actions))   #randomly select an action

        unofficially_explored_action_indices = [i for i,child in self.unrecognized_children]
        if index in unofficially_explored_action_indices:
            i = unofficially_explored_action_indices.index(index)
            return i

        action = self.officially_unexplored_actions[index]
        self.unrecognized_children.append([index, node(self, self.test_dir, action)])
        return -1


    def set_up_root_node(self, mobile_object_IDs, mobile_object_types, held_fixed_list, precomputed_pushing_points_and_point_pairs, view_matrix=None, proj_matrix=None):
        global image_num

        # open scene in root node
        scene_path = os.path.join(self.node_dir, "scene.csv")
        binID = p_utils.open_saved_scene(scene_path, self.node_dir, None, None, mobile_object_IDs, mobile_object_types, held_fixed_list)

        #get the COMs of the objects, which are needed for precomputed pushing points
        object_coms = []
        scene_data = file_handling.read_csv_file(scene_path,[str, float, float, float, float, float, float, float, float, float, float, int])
        for row in scene_data[1:]:
            object_coms.append(np.array([row[1],row[2],row[3]]))

        #set up precomputed pushing points for all objects in the scene
        for i in np.arange(len(mobile_object_IDs)):
            object_type = mobile_object_types[i]
            object_com = object_coms[i]
            if precomputed_pushing_points_and_point_pairs.get(object_type) is None:
                precomputed_pushing_points_and_point_pairs[object_type] = precompute_pushing_points_and_point_pairs(object_type, object_com)

        if view_matrix is not None:
            p_utils.print_image(view_matrix,proj_matrix,self.test_dir,image_num)

        self.officially_unexplored_actions =\
            self.generate_action_list(mobile_object_IDs, mobile_object_types, precomputed_pushing_points_and_point_pairs, view_matrix, proj_matrix)

        if view_matrix is not None:
            image_num+=1

        return binID


    def apply_action(self, dt, mobile_object_IDs, mobile_object_types, held_fixed_list, precomputed_pushing_points_and_point_pairs, binID, target_index,
                     view_matrix=None, proj_matrix=None):
        global image_num

        #do not repeat actions
        if self.node_reward is not None:
            return

        reward = 0.


        #open scene before
        p_utils.open_saved_scene_existing_objects(os.path.join(self.parent.node_dir, "scene.csv"), mobile_object_IDs)

        self.action_type, self.point_1, self.point_2 = self.action_to_get_here
        #action_type is either "push" or "grasp"
        #if it is "push", then point_1 is the pusher starting point, and point_2-point_1 forms the pushing vector
        #if it is "grasp", then point_1 and point_2 are the points of the grasp.

        unsuccessful_push = False

        #create cylinder
        cylinderID = p_utils.create_cylinder(pushing_point_free_space_radius, 0.05)
        p.resetBasePositionAndOrientation(cylinderID, self.point_1+cylinder_height_offset, (0., 0., 0., 1.))

        if self.action_type == "grasp":
            #TODO replace this with actual robot grasp?
            pass
        else:
            #push
            #TODO replace this with actual robot push?
            push_vector = self.point_2 - self.point_1
            push_vector = push_distance * push_vector / np.linalg.norm(push_vector)
            pusher_end = self.point_1 + push_vector + cylinder_height_offset
            cylinder_original_point, _ = p.getBasePositionAndOrientation(cylinderID)
            p_utils.push(pusher_end, cylinderID, dt, time_out=2.)
            cylinder_new_point, _ = p.getBasePositionAndOrientation(cylinderID)
            p_utils.let_time_pass(cylinderID, dt,mobile_object_IDs)

            #see if the push was successful or not, by seeing if the pusher moved even a tenth of the requested amount.
            actual_push_distance = np.linalg.norm(np.array(cylinder_new_point) - np.array(cylinder_original_point))
            if actual_push_distance < 0.1*push_distance:
                unsuccessful_push = True
                # since actual_push_distance < 0.1*push_distance, the push failed, so this node should not have been opened/existed.
                # Since this node should not have existed, it should at least be a dead end with no further actions to be taken.
                # By setting unsuccessful_push=True, this node is marked as a dead end.

        p.removeBody(cylinderID)

        #save scene after
        p_utils.save_scene(os.path.join(self.node_dir,"scene.csv"), binID, mobile_object_IDs, mobile_object_types, held_fixed_list)

        #reward function
        if self.action_type=="grasp":
            grasp_ray = p.rayTest(self.point_1, self.point_2)
            if len(grasp_ray) > 0:
                grasped_object_id = grasp_ray[0][0]
                if grasped_object_id == mobile_object_IDs[target_index]:
                    reward+=basic_reward

        if view_matrix is not None:
            p_utils.print_image(view_matrix,proj_matrix,self.test_dir,image_num)

        if self.action_type == "push":
            if not unsuccessful_push:
                self.officially_unexplored_actions =\
                    self.generate_action_list(mobile_object_IDs, mobile_object_types, precomputed_pushing_points_and_point_pairs, view_matrix, proj_matrix)
            else:
                print("push failed")

        if view_matrix is not None:
            image_num+=1

        self.node_reward = reward


    def back_propagate(self, reward):
        #update sum of rewards and number of times visited, for this node and for its parent nodes up the tree to the root.
        self.sum_of_rewards += reward
        self.number_of_visits += 1
        if self.parent != None:
            self.parent.back_propagate(reward_discount * reward)


    def generate_action_list(self, mobile_object_IDs, mobile_object_types, precomputed_pushing_points_and_point_pairs, view_matrix=None, proj_matrix=None):
        global image_num

        if self.depth == maximum_depth:
            return []

        candidate_list = []

        for i in np.arange(len(mobile_object_IDs)):
            id = mobile_object_IDs[i]
            position, orientation = p.getBasePositionAndOrientation(id)
            object_type = mobile_object_types[i]

            points, edges = precomputed_pushing_points_and_point_pairs[object_type]
            transformed_points = np.zeros_like(points)
            for point_index in np.arange(points.shape[0]):
                #precomputed pushing points are already transformed by COM when loaded, so only transforming them here by object position and orientation
                transformed_points[point_index] = p_utils.rotate_vector(points[point_index], orientation) + position

            #filter edges so that those edges on top of each other are not repeated. Save the median height for these overlaping edges
            filtered_edges = []
            heights = []
            for j in np.arange(len(edges)):
                point1_index, point2_index = edges[j]
                point_1 = transformed_points[point1_index]
                point_2 = transformed_points[point2_index]
                edge_already_added = False
                for k in np.arange(len(filtered_edges)):
                    point1_index_other, point2_index_other = filtered_edges[k]
                    point_1_other = transformed_points[point1_index_other]
                    point_2_other = transformed_points[point2_index_other]
                    horizontal_dist = np.linalg.norm(point_1[:2]-point_1_other[:2]) + np.linalg.norm(point_2[:2]-point_2_other[:2])
                    vertical_dist = np.abs(point_1[2]-point_1_other[2]) + np.abs(point_2[2]-point_2_other[2])
                    if horizontal_dist < 0.1*vertical_dist:
                        edge_already_added = True
                        heights[k].append(point_1[2])
                        break
                if not edge_already_added:
                    filtered_edges.append(edges[j])
                    heights.append([point_1[2]])
            #save median heights
            final_heights = []
            for k in np.arange(len(heights)):
                heights[k].sort()
                final_heights.append(heights[k][int(len(heights[k])/2)])

            #add points to candidate lists
            for j in np.arange(len(filtered_edges)):
                point1_index, point2_index = filtered_edges[j]

                # every single pushing point has a partner in an edge. Therefore, pushing points and their directions are calculated by edge.
                point_1 = transformed_points[point1_index]
                point_2 = transformed_points[point2_index]
                if np.linalg.norm(point_1[:2] - point_2[:2]) > 0.01:
                    point_1[2] = final_heights[j]
                    point_2[2] = final_heights[j]
                    candidate_list.append((point_1, point_2))
                    candidate_list.append((point_2, point_1))


        #create a test pusher cylinder for collision checking. The cylinder is placed at each candidate pushing point to test if it is valid. Invalid pushing points are filtered out.
        cylinderID = p_utils.create_cylinder(pushing_point_free_space_radius, 0.05)

        # filter the pushing points. If the cylinder, when placed at a pushing point, overlaps with an object, that point is out.
        # If both pushing points in a pair get added in, then add a grasp.
        filtered_candidate_list = []
        added_even=False
        for i,point_pair in enumerate(candidate_list):
            point, _ = point_pair #the first point of each pair is the point where the pusher will start, and therefore it is the point to check for collisions with objects.
            p.resetBasePositionAndOrientation(cylinderID, point+cylinder_height_offset, (0., 0., 0., 1.))
            p.performCollisionDetection()
            contact_results = p.getContactPoints(cylinderID)
            if len(contact_results) == 0:
                filtered_candidate_list.append(("push", candidate_list[i][0], candidate_list[i][1]))
                if i%2==0:
                    added_even=True
                elif added_even:
                    if np.linalg.norm(candidate_list[i][0] - candidate_list[i][1]) < 0.04:
                        filtered_candidate_list.append(("grasp", candidate_list[i][0], candidate_list[i][1]))
                    added_even=False
                else:
                    added_even=False
            else:
                added_even=False

        p.removeBody(cylinderID)


        if view_matrix is not None:
            #create spheres to show valid pushing points
            point_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=pushing_point_free_space_radius)
            point_collision_IDs = []
            for type_and_point_pair in filtered_candidate_list:
                _, point, _ = type_and_point_pair
                point_id = p.createMultiBody(baseCollisionShapeIndex=point_collision_shape, basePosition=point)
                point_collision_IDs.append(point_id)
            p_utils.print_image(view_matrix, proj_matrix, self.test_dir, image_num, "_pushing_points")
            #remove spheres
            for point_id in point_collision_IDs:
                p.removeBody(point_id)


        return filtered_candidate_list


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
        3a. run the action taken from its parent using apply_action, save the reward and the unexplored actions in the node. Set the current node to be n
        3b. Repeat while the current node's unexplored actions is not an empty list:
            3ba. from the current node's unexplored actions, create or open an unrecognized child using create_or_open_unrecognized_child,
                and set the current node to that node
            3bb. run the action taken from its parent using apply_action, and save the reward and the unexplored actions in the node.
                If the current node already existed, apply_action will not do anything.
        3c. update the sum of rewards and the number of times visited for node n and up its chain by calling back_propagate for node n,
            passing it the node reward of the last node called in the loop
Return the action_to_get_here of the direct child of the root that has the highest sum of rewards.

To create the root node, call the constructor, then set_up_root_node.



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
Step 3bb is run in parallel, picking up on the same processes used for 3a.
3c is run serially, to avoid race conditions in updating rewards and number of times visited up the chain.
'''
def MCTS(test_dir, dt, scene_file, target_index, view_matrix=None, proj_matrix=None):
    start_time = time.perf_counter_ns()

    mobile_object_IDs = []
    mobile_object_types = []
    held_fixed_list = []
    precomputed_pushing_points_and_point_pairs = {}

    root_node = node(None, test_dir)

    #load a scene to the node
    file_handling.copy_file(scene_file, os.path.join(root_node.node_dir, "scene.csv"))
    binID = root_node.set_up_root_node(mobile_object_IDs, mobile_object_types, held_fixed_list, precomputed_pushing_points_and_point_pairs, view_matrix, proj_matrix)
    print("root:", root_node.nodeNum, f'\t\t{len(root_node.officially_unexplored_actions)} actions')

    official_child_expansion_count = 0
    total_expansion_count = 0

    #if a direct child of the root node is a grasp on the target object, return it
    for action in root_node.officially_unexplored_actions:
        if action[0]=='grasp':
            grasp_ray = p.rayTest(action[1], action[2])
            if len(grasp_ray) > 0:
                grasped_object_id = grasp_ray[0][0]
                if grasped_object_id == mobile_object_IDs[target_index]:
                    print("Found a grasp action, returning it.")
                    return action

    while True:
        node_to_expand = root_node.select_node_to_expand(explore_factor)
        if len(node_to_expand.children)==0 and len(node_to_expand.officially_unexplored_actions)==0:
            break
        print("iteration",official_child_expansion_count)
        if official_child_expansion_count == iteration_limit:
            break
        official_child_expansion_count += 1
        total_expansion_count += 1

        node_to_expand.create_child()
        n = node_to_expand.children[-1]
        number_of_nodes = n.nodeNum + 1
        n.apply_action(dt, mobile_object_IDs, mobile_object_types, held_fixed_list, precomputed_pushing_points_and_point_pairs, binID, target_index, view_matrix, proj_matrix)
        print("child:",n.nodeNum,f'\t\t{len(n.officially_unexplored_actions)} actions')
        current_node = n

        #rollout
        while len(current_node.officially_unexplored_actions) != 0:
            total_expansion_count += 1

            unrecognized_child_index = current_node.create_or_open_unrecognized_child()

            '''#Modification to MCTS: if a grasp is found in a rollout, take it instead of a random move
            winning_index = current_node.get_winning_move_if_available(mobile_object_IDs, target_index)

            if winning_index == -1:
                unrecognized_child_index = current_node.create_or_open_unrecognized_child()
            else:
                unrecognized_child_index = current_node.create_or_open_unrecognized_child(winning_index)'''
            current_node = current_node.unrecognized_children[unrecognized_child_index][1]
            if current_node.nodeNum <= number_of_nodes:
                number_of_nodes = current_node.nodeNum + 1
            current_node.apply_action(dt, mobile_object_IDs, mobile_object_types, held_fixed_list, precomputed_pushing_points_and_point_pairs, binID, target_index,
                                      view_matrix, proj_matrix)
            print("unrecognized child:", current_node.nodeNum,f'\t\t{len(current_node.officially_unexplored_actions)} actions','\t\tgot from',current_node.action_to_get_here[0])

        n.back_propagate(current_node.node_reward)

        #print('\t\t\tTime to run so far:', (time.perf_counter_ns() - start_time) / 1e9, 's')

    print(f"expanded {official_child_expansion_count} recognized children")
    print(f"expanded {total_expansion_count} nodes in total")

    #return the child of the root node with the highest sum of rewards.
    action_to_take = None
    best_sum_of_rewards = 0
    for child in root_node.children:
        if child.sum_of_rewards >= best_sum_of_rewards:
            action_to_take = child.action_to_get_here
            best_sum_of_rewards = child.sum_of_rewards
    print("action to take in root node:",action_to_take)
    print("sum of rewards:",best_sum_of_rewards)

    print('Time to run:', (time.perf_counter_ns() - start_time) / 1e9, 's')

    return action_to_take

#For timing purposes. Copy where needed, perhaps to a single round of apply_action and generate_action_list:
#start_time = time.perf_counter_ns()
#print('Time to open scene:', (time.perf_counter_ns() - start_time) / 1e9, 's')
