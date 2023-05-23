import numpy as np
import pybullet as p
import pybullet_utilities as p_utils
import file_handling
import os

'''# make directory for simulation files
testNum = 1
while os.path.exists("test" + str(testNum)):
    testNum += 1
test_dir = "test" + str(testNum)
os.mkdir(test_dir)



#make images directory
imgs_dir = os.path.join(test_dir,"imgs")
os.mkdir(imgs_dir)'''

#physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
dt = 1./240.


#lwr_ID = p.loadURDF(os.path.join("robot models","kuka_lwr","kuka.urdf"))
robotID = p.loadSDF(os.path.join("robot models", "kuka_iiwa", "kuka_with_gripper2.sdf"))[0]
time_amount = 3.
count=0

for joint_index in np.arange(p.getNumJoints(robotID)):
    print(p.getJointState(robotID, joint_index))
    print(p.getJointInfo(robotID, joint_index))
    print()

def push(pos):
    joint_positions = p.calculateInverseKinematics(robotID, 7, pos, p.getQuaternionFromEuler((np.pi, 0., 0.)))
    print("joint positions:\n",joint_positions)
    p.setJointMotorControlArray(robotID, [i for i, pos in enumerate(joint_positions)], p.POSITION_CONTROL, joint_positions)

import time
push((.3,0.,0.3))
while time_amount > 0:
    time_val = count * dt
    count += 1

    p.stepSimulation()

    time.sleep(dt)
    time_amount -= dt

'''joints = []
for i in range(p.getNumJoints(lwr_ID)):
    info = p.getJointInfo(lwr_ID, i)
    joint_id = info[0]
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    if joint_type == p.JOINT_REVOLUTE:
        joints.append(joint_id)



#get gripper
gripper_ID = p.loadURDF()

ee_position, _ = get_link_pose(ur5e, ur5e_ee_id)
end_effector = p.loadURDF(
    "assets/ur5e/gripper/robotiq_2f_85.urdf",
    ee_position,
    p.getQuaternionFromEuler((0, -np.pi / 2, 0)),
)
ee_tip_offset = 0.1625
gripper_angle_open = 0.03
gripper_angle_close = 0.8
gripper_angle_close_threshold = 0.73
gripper_mimic_joints = {
    "left_inner_finger_joint": -1,
    "left_inner_knuckle_joint": -1,
    "right_outer_knuckle_joint": -1,
    "right_inner_finger_joint": -1,
    "right_inner_knuckle_joint": -1,
}
for i in range(p.getNumJoints(end_effector)):
    info = p.getJointInfo(end_effector, i)
    joint_id = info[0]
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    if joint_name == "finger_joint":
        gripper_main_joint = joint_id
    elif joint_name == "dummy_center_fixed_joint":
        ee_tip_id = joint_id
    elif "finger_pad_joint" in joint_name:
        p.changeDynamics(
            end_effector, joint_id, lateralFriction=0.9
        )
        ee_finger_pad_id = joint_id
    elif joint_type == p.JOINT_REVOLUTE:
        gripper_mimic_joints[joint_name] = joint_id
        # Keep the joints static
        p.setJointMotorControl2(
            end_effector, joint_id, p.VELOCITY_CONTROL, targetVelocity=0, force=0,
        )

ee_constraint = p.createConstraint(
    parentBodyUniqueId=lwr_ID,
    parentLinkIndex=ur5e_ee_id,
    childBodyUniqueId=end_effector,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=(0, 0, 1),
    parentFramePosition=(0, 0, 0),
    childFramePosition=(0, 0, -0.02),
    childFrameOrientation=p.getQuaternionFromEuler((0, -np.pi / 2, 0))
)

p.changeConstraint(ee_constraint, maxForce=10000)

# Set up mimic joints in robotiq gripper: left
c = p.createConstraint(
    end_effector,
    gripper_main_joint,
    end_effector,
    gripper_mimic_joints["left_inner_finger_joint"],
    jointType=pb.JOINT_GEAR,
    jointAxis=[1, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)
p.changeConstraint(c, gearRatio=1, erp=0.8, maxForce=10000)
c = p.createConstraint(
    end_effector,
    gripper_main_joint,
    end_effector,
    gripper_mimic_joints["left_inner_knuckle_joint"],
    jointType=pb.JOINT_GEAR,
    jointAxis=[1, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)
p.changeConstraint(c, gearRatio=-1, erp=0.8, maxForce=10000)
# Set up mimic joints in robotiq gripper: right
c = p.createConstraint(
    end_effector,
    gripper_mimic_joints["right_outer_knuckle_joint"],
    end_effector,
    gripper_mimic_joints["right_inner_finger_joint"],
    jointType=pb.JOINT_GEAR,
    jointAxis=[1, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)
p.changeConstraint(c, gearRatio=1, erp=0.8, maxForce=10000)
c = p.createConstraint(
    end_effector,
    gripper_mimic_joints["right_outer_knuckle_joint"],
    end_effector,
    gripper_mimic_joints["right_inner_knuckle_joint"],
    jointType=pb.JOINT_GEAR,
    jointAxis=[1, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)
p.changeConstraint(c, gearRatio=-1, erp=0.8, maxForce=10000)
# Set up mimic joints in robotiq gripper: connect left and right
c = p.createConstraint(
    end_effector,
    gripper_main_joint,
    end_effector,
    gripper_mimic_joints["right_outer_knuckle_joint"],
    jointType=pb.JOINT_GEAR,
    jointAxis=[0, 1, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
    physicsClientId=_client_id,
)
p.changeConstraint(c, gearRatio=-1, erp=0.8, maxForce=1000)'''