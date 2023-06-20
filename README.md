# RoboticsProject2023

## Current simulations folder

Some files are utilities that do not run simulations on their own. Other files are the main programs.

The main programs in the current simulation folder can be split into three categories:

- Monte Carlo Tree Search (MCTS) projects
- Center of mass (COM) projects
- other projects (either small prototypes or helper files that are not attached to an actual project yet)

All of them involve a cylindrical pusher that executes quasi-static pushes on various rigid body objects.

### MCTS projects

- scenario_with_MCTS.py

scenario_with_MCTS.py runs a motion planner using Monte Carlo Tree Search to find an efficient series of pushes to declutter a 3D scene, such that at the end the robot can grasp a target object in a bin.


### COM projects

- quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py
- COM_overlay.py
- COM_multiple.py
- COM_multiple_2.py
- blocks_rigid_bodies_differentiable.py
- YCB_push_several_COMs.py
- YCB_push_several_masses.py

quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py takes the text files from the object models folder and expands them into rigid body objects consisting of cubes linked together by fixed joints. Initially, all cubes are candidates for the location of the object's center of mass. At each loop, the simulation takes the region of candidate cubes and plots a pushing course that splits the region in half. The pusher then pushes the object along the course. Based on the object's direction, the center of mass cannot be in one of the halves, so the cubes in the ineligible half are removed from the candidate list. This is repeated until there is one cube remaining, its center is the simulation's candidate center of mass. The estimated center of mass is then compared to the ground truth center of mass. Results show good agreement between those two values for the simulated objects. The idea of using rotations and pushing courses to eliminate candidate regions and isolate the center of mass is based on Mason's voting theorem, but this version uses only the push direction, without taking the friction cone into account.

COM_overlay.py runs the same scenario as quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py, but on YCB objects instead of cubes. Different points along the object are selected to be candidate COM points, and the probability of each one being the center of mass changes as some are eliminated in each push.

COM_multiple.py searches for the centers of mass of multiple objects that are pushed all at once. Its approach is similar to that of a particle filter: many scenarios are generated, in each one each object has the same starting position but a different randomly generated center of mass. In all scenarios, the objects are pushed in the same way. The final poses of the objects in a scenario are compared with the final poses of the objects in the ground truth simulation, and the scenario is given an accuracy score. Centers of mass are weighted according to their accuracy score to guide a search for more accurate centers of mass.

COM_multiple_2.py takes a more systematic approach to searching for the centers of masss of a group of objects. Given a scene and a ground truth simulation, it starts with a guess for the center of mass of each of the objects. It then does a simulation, after which it has the poses of the objects before starting, after the ground truth simulation, and after the new simulation. It generates test points around the COM, and checks the amount they moved in the ground truth vs the amount they moved in the new simulation. Points that moved more than expected have too little mass, they attract the COM. Points that moved less than expected have too much mass, they repel the COM. Using the test points' data, a new COM is calculated. A new simulation can then be taken, which allows repeating the cycle. The COM is iteratively improved until a maximum number of iterations is reached, or some error value goes below threshold.

blocks_rigid_bodies_differentiable.py uses block objects, like quasistatic_quasi-mason_simplified_rigid_body_pybullet_sim.py. It pushes objects made up of blocks, each time in the same way. It finds the masses and frictions of the blocks such that the objects they make up have a motion matching the ground truth push. The center of mass is the weighted average the block locations, weighted by mass.

YCB_push_several_COMs.py and YCB_push_several_masses.py are both files written earlier to test the effects of a YCB object's inertial properties on its motion. Both take several copies of a YCB object, remove collision detection between those copies, and set them to be superimposed on each other. Each copy has a varying value of an inertial property: center of mass for the former simulation and total mass for the latter simulation. The pusher pushes all copies simulataneously, allowing the user to see how variation in a property affects the motion. So far, results indicate that the center of mass affects the motion of an object undergoing quasi-static pushing, but total mass has no effect.


### other projects
- process_robot_lab_data.py
- YCB_several_pushes.py
- YCB_bin1.py
- YCB_bin_scene_generate.py
- YCB_scene_generate.py
- YCB_bin_scene_load.py

process_robot_lab_data.py is an unattached helper file. It takes data from the real life robot, which consists of end effector coordinates, forces (so far unused), and two poses of an object. The poses are before and after a push, while the end effector coordinates were recorded before, during, and after the same push. The frame-by-frame object poses are interpolated and displayed, while the robot end effector is simulated by the cylindrical pusher. This work is incomplete, since the start time of the object pose is calculated by taking the minimum pusher z-coordinate (height), and the end effector coordinates are of the robot arm's hand rather than its finger.

YCB_several_pushes.py pushes a single YCB object in a different location in each push for several pushes, to see how the object reacts and to allow the user to see the center of mass. The simulation is reset between pushes.

YCB_bin1.py is a test simulation with several YCB objects.

YCB_bin_scene_generate.py generates a scene csv file, listing the starting objects, their centers of mass, their poses (position and orientation), and whether or not they are held fixed.

YCB_scene_generate.py generates a scene csv file without a fixed bin object.

YCB_bin_scene_load.py opens up a saved scene via its csv file.



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
