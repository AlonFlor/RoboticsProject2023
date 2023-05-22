import numpy as np
import pybullet as p
import pybullet_utilities as p_utils
import file_handling
import os


physicsClient = p.connect(p.DIRECT)
#physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.

mobile_object_IDs = []
mobile_object_types = []
held_fixed_list = []

view_matrix, proj_matrix = p_utils.set_up_camera((0.,0.,0.), 0.75, 0, -75)



class tree_node:
    def __init__(self,parent):
        self.parent = parent
        self.node_reward = 0.
        self.children = []



        # make directory for simulation files
        self.nodeNum = 1
        while os.path.exists("node" + str(self.nodeNum)):
            self.nodeNum += 1
        self.node_dir = "node" + str(self.nodeNum)
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
            #create second cylinder for grasping
            cylinder2ID = p.createMultiBody(1., cylinder_shapeID, cylinder_visual_shapeID, (0., 0., 0.5), (0., 0., 0., 1.))

            p.resetBasePositionAndOrientation(cylinder2ID, self.point_2, (0., 0., 0., 1.))

            #grasp
            p_utils.grasp()

            p.removeBody(cylinder2ID)

        else:
            #push
            pusher_end = self.point_2
            p_utils.push(pusher_end, cylinderID, mobile_object_IDs, dt)

        p.removeBody(cylinderID)

        #save scene after
        p_utils.save_scene(os.path.join(self.node_dir,"scene.csv"), binID, mobile_object_IDs, mobile_object_types, held_fixed_list)
        p_utils.write_PLY_files(os.path.join(self.node_dir,str(self.nodeNum)), view_matrix, proj_matrix, mobile_object_IDs)

    def calculate_and_set_reward(self):
        #calculate reward and set it. Then update rewards of parent nodes up the chain.
        pass


    def generate_action_list(self):
        self.actions = []

        #popen call to C++ script to generate PLY file with points from PLY file generated by this node.
        #then each edge is two possible pushing actions (point_1, point_2) and (point_2, point_1)
        #points not assigned to edges need a normal to generate their counterpart point for pushing.

        #The C++ code should be changed so that all points include the normals assigned to those points.
        #points not assigned to edges will be given the center of the object as their counterpart point for now, but this is not a good long term arrangement because of long objects.
        #Use the placeholder method to make the code run while waiting for updates to the C++ code.


'''
Algorithm:
1. generate a node using the constructor
2. then run the action taken from its parent's action list in apply_action
3. then find and apply the reward for that node and up its chain in calculate_and_set_reward
4. then generate a new set of nodes using generate_action_list
5. from all unexplored actions in all nodes, select an action.

the root node can only do 1., 4., and 5.
when parallelizing this code, 1., 2., and 4. can be done in parallel. We would need to change 5. to output a list of actions that will be explored simultaneously rather than one action.
'''

p.disconnect()
