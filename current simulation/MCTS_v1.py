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
    def __init__(self,parent):
        self.parent = parent
        self.node_reward = 0.
        self.children = []

        # make directory for simulation files
        self.nodeNum = 0
        node_dir_path = os.path.join(test_dir,"node_" + str(self.nodeNum))
        while os.path.exists(node_dir_path):
            self.nodeNum += 1
            node_dir_path = os.path.join(test_dir,"node_" + str(self.nodeNum))
        self.node_dir = node_dir_path
        os.mkdir(self.node_dir)

    def apply_action(self, action):
        self.action_type, self.point_1, self.point_2 = action
        #action_type is either "push" or "grasp"
        #if it is "push", then point_1 is the pusher starting point, and point_2-point_1 forms the pushing vector
        #if it is "grasp", then point_1 and point_2 are the points of the grasp.

        #open scene before
        binID = p_utils.open_saved_scene(os.path.join(self.parent.node_dir, "scene.csv"), self.node_dir, [], [], mobile_object_IDs, mobile_object_types, held_fixed_list)

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
            self.node_reward = 100. #congratulations on being able to grasp!

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
        p_utils.write_PLY_files(self.node_dir, view_matrix, proj_matrix, mobile_object_IDs)

        p.resetSimulation()

    def cascade_reward_upwards(self):
        #update rewards of parent nodes up the chain.
        #TODO this might be the wrong way to go about it. Please double-check
        if self.parent != None:
            self.parent.node_reward += reward_discount * self.node_reward
            self.parent.cascade_reward_updards()


    def generate_action_list(self):
        self.actions = []

        candidate_list = []

        for id in mobile_object_IDs:
            basic_id_ply_str = "objID_"+str(id)

            id_ply_str = os.path.join(self.node_dir, basic_id_ply_str+".ply")
            id_ply_obstacles_str = os.path.join(self.node_dir, basic_id_ply_str+"_obstacles.ply")
            id_ply_with_normals_str = os.path.join(self.node_dir, basic_id_ply_str+"_with_normals.ply")
            id_ply_obstacles_with_normals_str = os.path.join(self.node_dir, basic_id_ply_str+"_obstacles_with_normals.ply")

            pcl_get_normals_command = "candidate pushing points code\\out\\build\\x64-Release\\pclnormal.exe " + id_ply_str + " " + id_ply_with_normals_str
            os.popen(pcl_get_normals_command)
            time.sleep(0.001)
            pcl_get_normals_command = "candidate pushing points code\\out\\build\\x64-Release\\pclnormal.exe " + id_ply_obstacles_str + " " + id_ply_obstacles_with_normals_str
            os.popen(pcl_get_normals_command)
            time.sleep(0.1)

            id_ply_pushing_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_pushing_points.ply")
            pcl_pushing_points_command = "candidate pushing points code\\out\\build\\x64-Release\\pclpush.exe "\
                                         + id_ply_with_normals_str + " " + id_ply_obstacles_with_normals_str + " " + id_ply_pushing_points_str
            os.popen(pcl_pushing_points_command)
            time.sleep(0.1)

            id_ply_downsampled_pushing_points_str = os.path.join(self.node_dir, basic_id_ply_str+"_downsampled_pushing_points.ply")
            pcl_down_sample_command = "candidate pushing points code\\out\\build\\x64-Release\\pclsample.exe "\
                                      + id_ply_pushing_points_str + " 30 " + id_ply_downsampled_pushing_points_str
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


'''
Algorithm:
1. generate a node using the constructor
2. then run the action taken from its parent's action list in apply_action, and find the resulting reward
3. then apply the reward for that node up its chain in cascade_reward_upwards
4. then generate a new set of nodes using generate_action_list
5. from all unexplored actions in all nodes, select an action.

the root node can only do 1., 4., and 5.
when parallelizing this code, 1., 2., and 4. can be done in parallel. We would need to change 5. to output a list of actions that will be explored simultaneously rather than one action.
'''

node0 = node(None)
#load a scene to the node
file_handling.copy_file(os.path.join("scenes",f"scene_{9}_shifted_COM.csv"), os.path.join(node0.node_dir, "scene.csv"))

node1 = node(node0)
start = np.array([0.15,0.,0.02])
end = np.array([start[0]-0.25, start[1], start[2]])
node1.apply_action(("push", start, end))
node1.generate_action_list()

p.disconnect()
