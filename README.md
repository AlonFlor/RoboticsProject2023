# RoboticsProject2023


## Old simulations folder

The old simulations folder contains simulations where a robot arm or related placeholder end effector pushes an object, and a regressions are done on the collected data to estimate the friction torque and center of mass. The data is time step by time step.

rigid_body_pybullet_sim.py uses a simulated robot arm to push the object. I found that it is more difficult to control the simulated arm than the real one, and this simulation was unsuccessful.

The other two simulations in the old simulation folder are quasistatic_rigid_body_pybullet_sim.py and quasistatic_rigid_body_pybullet_sim_2.py. Both of them attempt controlled quasi-static pushes of the object.

The former uses an abstract force, which is constant at first, and then is set equal to the combined friction force at the given time step. This simulation was unsuccessful.

The latter uses a floating cylinder moving at a constant velocity, which represents the robot arm's end effector but without the artifacts of simulating the robot arm directly. This simulation was rather successful, in that the regressions generally approximated the location of the center of mass when the rotation motion had a low acceleration and/or no effective trend in angular velocity in the time step by time step graph. When the angular velocity had a clear trend, indicating angular acceleration despite the quasi-static assumption, the model was only able to approximate the center of mass after adjusting for that acceleration. This latter scenario generally happened with pushes on longer objects such as the hammer and wrench.

The file tests.zip, also located in the old simulations folder, consists of a run of quasistatic_rigid_body_pybullet_sim_2.py on all of the objects in the object models folder.

Even though quasistatic_rigid_body_pybullet_sim_2.py was somewhat successful, I consider it an old simulation because the attempt to recover the center of mass based on time step by time step data is contrary to Changkyu's approach. According to Professor Boularias, Changkyu assumed perfect quasi-static motion, taking only the start and end poses and interpolating the time steps in-between. His algorithm then attempted to predict the interpolated data by shifting the mass and friction in the differentiable model.



## Running the files

The models should be able to run by calling them from any software that runs Python 3 (Python 3.9 is the version used to program the simulations in PyCharm). The requisite libraries are numpy, sk_learn, os, time, PyBullet, and matplotlib.
