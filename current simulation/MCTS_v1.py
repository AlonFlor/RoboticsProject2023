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

    def create_unrecognized_child(self):
        #Create a child in simulation
        index = random.randrange(len(self.officially_unexplored_actions))   #randomly select an action
        action = self.officially_unexplored_actions[index]
        self.unrecognized_children.append([index, node(self, action)])

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

        p_utils.write_PLY_files(self.node_dir, view_matrix, proj_matrix, mobile_object_IDs)
        p.resetSimulation()

        return reward

    def back_propagate(self, reward):
        #update sum of rewards and number of times visited, for this node and for its parent nodes up the tree to the root.
        self.sum_of_rewards += reward
        self.number_of_visits += 1
        if self.parent != None:
            self.parent.back_propagate(reward_discount * reward)


    def generate_action_list(self):
        candidate_list = []

        for id in mobile_object_IDs:
            basic_id_ply_str = "objID_"+str(id)

            id_ply_str = os.path.join(self.node_dir, basic_id_ply_str+".ply")
            id_ply_obstacles_str = os.path.join(self.node_dir, basic_id_ply_str+"_obstacles.ply")
            id_ply_with_normals_str = os.path.join(self.node_dir, basic_id_ply_str+"_with_normals.ply")
            id_ply_obstacles_with_normals_str = os.path.join(self.node_dir, basic_id_ply_str+"_obstacles_with_normals.ply")

            pcl_get_normals_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclnormal.exe\" " + id_ply_str + " " + id_ply_with_normals_str
            print(pcl_get_normals_command)
            os.popen(pcl_get_normals_command)
            time.sleep(0.1)
            pcl_get_normals_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclnormal.exe\" " + id_ply_obstacles_str + " " + id_ply_obstacles_with_normals_str
            print(pcl_get_normals_command)
            os.popen(pcl_get_normals_command)
            time.sleep(0.1)

            id_ply_pushing_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_pushing_points.ply")
            pcl_pushing_points_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclpush.exe\" "\
                                         + id_ply_with_normals_str + " " + id_ply_obstacles_with_normals_str + " " + id_ply_pushing_points_str
            print(pcl_pushing_points_command)
            os.popen(pcl_pushing_points_command)
            time.sleep(0.1)

            id_ply_downsampled_pushing_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_downsampled_pushing_points.ply")
            pcl_down_sample_command = "\"candidate pushing points code\\out\\build\\x64-Release\\pclsample.exe\" "\
                                      + id_ply_pushing_points_str + " 30 " + id_ply_downsampled_pushing_points_str
            print(pcl_down_sample_command)
            os.popen(pcl_down_sample_command)
            time.sleep(0.1)

            candidate_list.append(p_utils.get_points_from_ply_files(id_ply_downsampled_pushing_points_str))

        print(candidate_list)

        #TODO popen call to C++ script to generate PLY file with points from PLY file generated by this node.
        #TODO then each edge is two possible pushing actions (point_1, point_2) and (point_2, point_1)
        #TODO points not assigned to edges need a normal to generate their counterpart point for pushing.

        #TODO The C++ code should be changed so that all points include the normals assigned to those points.
        #TODO points not assigned to edges will be given the center of the object as their counterpart point for now, but this is not a good long term arrangement because of long objects.
        #TODO Use the placeholder method to make the code run while waiting for updates to the C++ code.

        #TODO The current method is extremely wasteful. I am re-generating mostly the same obstacles files, which get assigned mostly the same normals.
        #TODO Have Eric change his code so that a single PLY file come in, with points labeled by object ID or an ID for the background, and out comes a PLY file with points and edges.

        #TODO there would be a depth limit. Nodes at the depth limit have no actions.

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
        3a. run the action taken from its parent using apply_action, and save the reward in a variable
        3b. then generate a new set of unexplored actions using generate_action_list.
        3c. Repeat:
            3ca. from the current node's unexplored actions, create an unrecognized child using create_unrecognized_child, and temporarily make that node the new current node
            3cb. run the action taken from its parent using apply_action, and save the reward in a variable
            3cc. generate a new set of unexplored actions using generate_action_list. If that list is empty, end the loop.
        3d. take the reward saved from 3a or 3cb (there should only be one variable that gets overwritten whenever 3cb is called), and set node_reward for n to that value
        3e. then update the sum of rewards and the number of times visited for node n and up its chain by calling back_propagate for node n
Return the child of the root with the highest reward.

To create the root node, call the constructor, then apply_action, and then generate_action_list.



Parallelization policy:
Step 1 is run serially.
    The algorithm is changed to branch by returning multiple nodes each time, and keeping track of the number of unexplored actions (not number of nodes).
    Once the number of unexplored actions collected is equal to the number of processors, or there are no more unexplored actions to collect, then finish this step.
    If the number of actions collected is 0, exit the loop like before.
Step 2 is run serially, to avoid collisions/race conditions in numbering the node files.
    This step is done for each unexplored action in each node in the list of nodes returned by step one.
    It returns a list of newly created child nodes for steps 3a and 3b.
Step 3a and 3b are run in parallel, one process per node from the list from step 2.
Step 3ca is run serially, to avoid collisions/race conditions in numbering the node files.
Steps 3cb and 3cc are run in parallel, picking up on the same processes used for 3a and 3b.
3d is is run in parallel, picking up on the same processes used for 3a, 3b, 3cb, and 3cc.
3e is run serially, to avoid race conditions in updating rewards and number of times visited up the chain.
'''

root_node = node(None)
#load a scene to the node
file_handling.copy_file(os.path.join("scenes",f"scene_{9}_shifted_COM.csv"), os.path.join(root_node.node_dir, "scene.csv"))
root_node.apply_action()
root_node.generate_action_list()
#root_node.create_child()

'''start = np.array([0.15,0.,0.02])
end = np.array([start[0]-0.25, start[1], start[2]])
node1 = node(root_node, ("push", start, end))
node1.apply_action()
node1.generate_action_list()'''

p.disconnect()
