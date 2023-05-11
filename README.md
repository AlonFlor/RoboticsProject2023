# RoboticsProject2023

## Current simulations folder

The programs in the current simulation folder are

- quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py
- process_robot_lab_data.py
- YCB_push_several_COMs.py
- YCB_push_several_masses.py
- YCB_several_pushes.py

All of them involve a cylindrical pusher that executes quasi-static pushes on various rigid body objects.

quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py takes the text files from the object models folder and expands them into rigid body objects consisting of cubes linked together by fixed joints. Initially, all cubes are candidates for the location of the object's center of mass. At each loop, the simulation takes the region of candidate cubes and plots a pushing course that splits the region in half. The pusher then pushes the object along the course. Based on the object's direction, the center of mass cannot be in one of the halves, so the cubes in the ineligible half are remved from the candidate list. This is repeated until there is one cube remaining, its center is the simulation's candidate center of mass. The estimated center of mass is then compared to the ground truth center of mass. Results show good agreement between those two values for the simulated objects. The idea of using rotations and pushing courses to eliminate candidate regions and isolate the center of mass is based on Mason's voting theorem, but this version uses only the push direction, without taking the friction cone into account.

process_robot_lab_data.py takes data from the real life robot, which consists of end effector coordinates, forces (so far unused), and two poses of an object. The poses are before and after a push, while the end effector coordinates were recorded before, during, and after the same push. The frame-by-frame object poses are interpolated and displayed, while the robot end effector is simulated by the cylindrical pusher. This work is incomplete, since the start time of the object pose is calculated by taking the minimum pusher z-coordinate (height), and the end effector coordinates are of the robot arm's hand rather than its finger.

YCB_push_several_COMs.py and YCB_push_several_masses.py both deal with the effects of a YCB object's inertial properties on its motion. Both take several copies of a YCB object, remove collision detection between those copies, and set them to be superimposed on each other. Each copy has a varying value of an inertial property: center of mass for the former simulation and total mass for the latter simulation. The pusher pushes all copies simulataneously, allowing the user to see how variation in a property affects the motion. So far, results indicate that the center of mass affects the motion of an object undergoing quasi-static pushing, but total mass has no effect. 

YCB_several_pushes.py pushes a single YCB object in a different location in each push for several pushes, to see how the object reacts and to allow the user to see the center of mass. The simulation is reset between pushes.

## Old simulations folder

The old simulations folder contains simulations where a robot arm or related placeholder end effector pushes an object, and a regressions are done on the collected data to estimate the friction torque and center of mass. The data is time step by time step.

rigid_body_pybullet_sim.py uses a simulated robot arm to push the object. I found that it is more difficult to control the simulated arm than the real one, and this simulation was unsuccessful.

The other two simulations in the old simulation folder are quasistatic_rigid_body_pybullet_sim.py and quasistatic_rigid_body_pybullet_sim_2.py. Both of them attempt controlled quasi-static pushes of the object.

The former uses an abstract force, which is constant at first, and then is set equal to the combined friction force at the given time step. This simulation was unsuccessful.

The latter uses a floating cylinder moving at a constant velocity, which represents the robot arm's end effector but without the artifacts of simulating the robot arm directly. This simulation was rather successful, in that the regressions generally approximated the location of the center of mass when the rotation motion had a low acceleration and/or no effective trend in angular velocity in the time step by time step graph. When the angular velocity had a clear trend, indicating angular acceleration despite the quasi-static assumption, the model was only able to approximate the center of mass after adjusting for that acceleration. This latter scenario generally happened with pushes on longer objects such as the hammer and wrench.

The file tests.zip, also located in the old simulations folder, consists of a run of quasistatic_rigid_body_pybullet_sim_2.py on all of the objects in the object models folder.

Even though quasistatic_rigid_body_pybullet_sim_2.py was somewhat successful, I consider it an old simulation because the attempt to recover the center of mass based on time step by time step data is contrary to Changkyu's approach. According to Professor Boularias, Changkyu assumed perfect quasi-static motion, taking only the start and end poses and interpolating the time steps in-between. His algorithm then attempted to predict the interpolated data by shifting the mass and friction in the differentiable model.



## Running the files

The models should be able to run by calling them from any software that runs Python 3 (Python 3.9 is the version used to program the simulations in PyCharm). The requisite libraries are numpy, os, PyBullet, and PIL. The simulations in the old simulations folder do not require PIL, but do require time, sk_learn, and matplotlib. quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py also requires the time library.
