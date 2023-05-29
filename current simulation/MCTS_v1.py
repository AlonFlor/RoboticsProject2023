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

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 0, -75)

mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []

push_distance = 0.1
reward_discount = 0.9

#pushing_points_per_object = 40.
precomputed_pushing_points_and_point_pairs = {}
pushing_point_free_space_radius = 0.015 / 2
explore_factor = 1.#0.5

maximum_depth = 5

image_num = 0

def precompute_pushing_points_and_point_pairs(object_type, object_com):
    #Points should be an np array, edges should be a list of indices.
    #TODO: replace this with more systematic code
    precomputed_points_file = os.path.join("object models",object_type,"precomputed_pushing_points.csv")
    precomputed_pushing_points = np.array(file_handling.read_csv_file(precomputed_points_file, [float, float, float]))
    precomputed_pushing_points -= object_com
    edges = [(0,1),(2,3),(4,5)]
    return precomputed_pushing_points,edges


class node:
    def __init__(self,parent, action=None):
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
        self.children.append(node(self, action))


    def create_or_open_unrecognized_child(self):
        #This function is for simulation rollouts. Create a child if it does not exist. Return its index if it did already exist, -1 otherwise.
        index = random.randrange(len(self.officially_unexplored_actions))   #randomly select an action

        unofficially_explored_action_indices = [i for i,child in self.unrecognized_children]
        if index in unofficially_explored_action_indices:
            i = unofficially_explored_action_indices.index(index)
            return i

        action = self.officially_unexplored_actions[index]
        self.unrecognized_children.append([index, node(self, action)])
        return -1


    def set_up_root_node(self):
        # open scene in root node
        scene_path = os.path.join(self.node_dir, "scene.csv")
        global binID
        binID = p_utils.open_saved_scene(scene_path, self.node_dir, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

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

        '''#TODO: delete this separated block of code
        global image_num
        p_utils.print_image(view_matrix,proj_matrix,test_dir,image_num)
        image_num+=1'''

        self.officially_unexplored_actions = self.generate_action_list()


    def apply_action(self):
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
        cylinder_radius = 0.01
        cylinder_height = 0.1
        cylinder_shapeID = p.createCollisionShape(p.GEOM_CYLINDER, radius=cylinder_radius, height=cylinder_height)
        cylinder_visual_shapeID = p.createVisualShape(p.GEOM_CYLINDER, radius=cylinder_radius, length=cylinder_height)
        cylinderID = p.createMultiBody(1., cylinder_shapeID, cylinder_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))

        p.resetBasePositionAndOrientation(cylinderID, self.point_1, (0., 0., 0., 1.))

        if self.action_type == "grasp":
            #TODO replace this with actual robot grasp?
            reward = 100. #congratulations on being able to grasp!
        else:
            #push
            #TODO replace this with actual robot push?
            push_vector = self.point_2 - self.point_1
            push_vector = push_distance * push_vector / np.linalg.norm(push_vector)
            pusher_end = self.point_1 + push_vector
            cylinder_original_point, _ = p.getBasePositionAndOrientation(cylinderID)
            p_utils.push(pusher_end, cylinderID, dt, time_out=2.)
            cylinder_new_point, _ = p.getBasePositionAndOrientation(cylinderID)

            #see if the push was successful or not, by seeing if the pusher moved even a tenth of the requested amount.
            actual_push_distance = np.linalg.norm(np.array(cylinder_new_point) - np.array(cylinder_original_point))
            if actual_push_distance < 0.1*push_distance:
                reward = -10
                unsuccessful_push = True
                # since actual_push_distance < 0.1*push_distance, the push failed, so this node should not have been opened/existed.
                # Since this node should not have existed, it should at least be a dead end with no further actions to be taken.
                # By setting unsuccessful_push=True, this node is marked as a dead end.

        p.removeBody(cylinderID)

        #save scene after
        global binID
        p_utils.save_scene(os.path.join(self.node_dir,"scene.csv"), binID, mobile_object_IDs, mobile_object_types, held_fixed_list)

        #TODO: delete this separated block of code
        target_pos, _ = p.getBasePositionAndOrientation(mobile_object_IDs[0])
        if np.linalg.norm(np.array(target_pos) - np.array([-0.1, 0., target_pos[2]])) < 0.03:
            reward+=100.
        '''global image_num
        p_utils.print_image(view_matrix,proj_matrix,test_dir,image_num)
        image_num+=1'''

        if self.action_type == "push":
            if not unsuccessful_push:
                self.officially_unexplored_actions = self.generate_action_list()
            else:
                print("push failed")

        self.node_reward = reward


    def back_propagate(self, reward):
        #update sum of rewards and number of times visited, for this node and for its parent nodes up the tree to the root.
        self.sum_of_rewards += reward
        self.number_of_visits += 1
        if self.parent != None:
            self.parent.back_propagate(reward_discount * reward)


    def generate_action_list(self):
        #global image_num
        if self.depth == maximum_depth:
            #image_num+=9 #TODO delete this when taking these images is no longer necessary
            return []

        candidate_list = []

        for i in np.arange(len(mobile_object_IDs)):
            id = mobile_object_IDs[i]
            position, orientation = p.getBasePositionAndOrientation(id)
            object_type = mobile_object_types[i]

            points, edges = precomputed_pushing_points_and_point_pairs[object_type]
            transformed_points = np.zeros_like(points)
            for point_index in np.arange(points.shape[0]):
                transformed_points[point_index] = p_utils.rotate_vector(points[point_index], orientation) + position
                #precomputed pushing points are already transformed by COM when loaded, so only transforming them here by object position and orientation
            for point1_index, point2_index in edges:
                # every single pushing point has a partner in an edge. Therefore, pushing points and their directions are calculated by edge.
                point_1 = transformed_points[point1_index]
                point_2 = transformed_points[point2_index]
                if point_1[2]>pushing_point_free_space_radius:
                    candidate_list.append((point_1, point_2))
                if point_2[2]>pushing_point_free_space_radius:
                    candidate_list.append((point_2, point_1))

        #add the pushing points as shapes for collision checking. Invalid pushing points are filtered out.
        point_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=pushing_point_free_space_radius)
        point_collision_IDs = []
        for i,point_pair in enumerate(candidate_list):
            point, _ = point_pair #the first point of each pair is the point where the pusher will start, and therefore it is the point to check for collisions with objects.
            point_id = p.createMultiBody(baseCollisionShapeIndex=point_collision_shape, basePosition=point)
            point_collision_IDs.append(point_id)
        p.performCollisionDetection()

        '''#global image_num
        #TODO delete this when taking these images is no longer necessary
        p_utils.print_image(view_matrix, proj_matrix, test_dir, image_num)
        image_num+=1'''

        #filter the pushing points. If any pushing point's sphere overlaps with an object, that point is out. Pushing points may overlap with each other.
        #TODO: check that this works. Pushing points fully enclosed by objects should have contact with them. contact[2] should give the id of the second body.
        filtered_candidate_list = []
        list_to_export = []
        for i,point_id in enumerate(point_collision_IDs):
            contact_results = p.getContactPoints(point_id)
            if len(contact_results)==0:
                add_point=True
            else:
                add_point = True
                for contact in contact_results:
                    if contact[2] not in point_collision_IDs:
                        add_point = False
                        break
            if add_point:
                filtered_candidate_list.append(("push", candidate_list[i][0], candidate_list[i][1]))
                list_to_export.append(candidate_list[i][0])
        #delete all pushing point spheres
        for point_id in point_collision_IDs:
            p.removeBody(point_id)

        #TODO add a function to check if two accepted points who are in the same edge can be a grasp action.
        # Criteria are that both points are within a maximum distance, and both points are not too close to the center of particularly large faces.
        # The second criterion might be pre-computed, in which case the edge would be pre-marked as "cannot be used for a grasp" (the actual designation would be shorter).

        '''#global image_num
        #TODO delete this when taking these images is no longer necessary
        p_utils.print_image(view_matrix, proj_matrix, test_dir, image_num)
        image_num+=8'''

        pushing_points_ply_path_str = os.path.join(self.node_dir, "pushing_points.ply")
        list_to_export = np.array(list_to_export)
        np.savetxt(pushing_points_ply_path_str, list_to_export, fmt='%f', header=p_utils.PLY_header_str(len(list_to_export)), comments='', encoding='utf-8')

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
start_time = time.perf_counter_ns()

root_node = node(None)
#load a scene to the node
file_handling.copy_file(os.path.join("scenes",f"scene_{9}_shifted_COM.csv"), os.path.join(root_node.node_dir, "scene.csv"))
root_node.set_up_root_node()

official_child_expansion_count = 0
total_expansion_count = 0
official_child_redo_count = 0


while True:
    node_to_expand = root_node.select_node_to_expand(explore_factor)
    if len(node_to_expand.children)==0 and len(node_to_expand.officially_unexplored_actions)==0:
        break
    official_child_expansion_count += 1
    total_expansion_count += 1

    node_to_expand.create_child()
    n = node_to_expand.children[-1]
    print("child:",n.nodeNum)
    if n.nodeNum < total_expansion_count:
        official_child_redo_count+=1
    n.apply_action()
    current_node = n
    while len(current_node.officially_unexplored_actions) != 0:
        total_expansion_count += 1

        unrecognized_child_index = current_node.create_or_open_unrecognized_child()
        current_node = current_node.unrecognized_children[unrecognized_child_index][1]
        print("unrecognized child:", current_node.nodeNum)
        current_node.apply_action()
    n.back_propagate(current_node.node_reward)

print(f"expanded {official_child_expansion_count} recognized children")
print(f"expanded {total_expansion_count} nodes in total")
print("Note: for both of these numbers, repeats were not factored out")
print("Estimate for nodes officially expanded more than once:",official_child_redo_count)

action_to_take = None
best_sum_of_rewards = 0
for child in root_node.children:
    if child.sum_of_rewards > best_sum_of_rewards:
        action_to_take = child.action_to_get_here
        best_sum_of_rewards = child.sum_of_rewards
print("action to take in root node:",action_to_take)
print("sum of rewards:",best_sum_of_rewards)

p.disconnect()

print('Time to run:', (time.perf_counter_ns() - start_time) / 1e9, 's')

#For timing purposes. Copy where needed, perhaps to a single round of apply_action and generate_action_list:
#start_time = time.perf_counter_ns()
#print('Time to open scene:', (time.perf_counter_ns() - start_time) / 1e9, 's')
