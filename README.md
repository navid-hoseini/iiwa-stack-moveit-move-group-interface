## Move group python interface for IIWA STACK
* This repository is a modified version of the original IIWA STACK repository [1, 2]. A move group python interface has been added to the original repository that can control the simulated KUKA iiwa robot via MoveIt package provided by the original repository.

* The code has been test on Ubuntu 18.04 x64 and ROS Melodic.

* To run the code follow the steps below:
  * Open a new terminal. Navigate to the root of the this repository, build the package using the command: `$ catkin_make`
  * Source the package using the command: `$ source devel/setup.bash `
  * Run the IIWA-STACK simulation: `$ roslaunch iiwa_moveit moveit_planning_execution.launch`
  * Open a new terminal and navigate to the root of the this repository. Source the package. Then run the following command to launch move group interface for simulated KUKA iiwa robot:
  * Now you should see command prompt like this: `$Input ee x y z delimited with space:`
  * Enter the desired x, y, z position for the robot end effector with the following format: '0 0.4 0'. Please mind the single quotation marks. Note that x, y, and z coordinates are separated with a single space. After entering the desired coordinates, just hit enter to send the command to MoveIt via move group interface. If everything has went OK, you should be able to see the robot move in MoveIt as well as Gazebo.
  * To terminate the program use Ctrl+c each of the two terminals.


* Contact: hhnavid@yahoo.com

\[1] [https://github.com/IFL-CAMP/iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack)

\[2] `@article{hennersperger2017towards,
  title={Towards MRI-based autonomous robotic US acquisitions: a first feasibility study},
  author={Hennersperger, Christoph and Fuerst, Bernhard and Virga, Salvatore and Zettinig, Oliver and Frisch, Benjamin and Neff, Thomas and Navab, Nassir},
  journal={IEEE transactions on medical imaging},
  volume={36},
  number={2},
  pages={538--548},
  year={2017},
  publisher={IEEE}
}`
