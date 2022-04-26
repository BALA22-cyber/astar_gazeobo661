# astarpart2
Baseline package for astar_gazebo661.

In the following instructions, we assume your catkin workspace is named catkin_ws and is located in your home directory.

- `cd ~/catkin_ws/src`
- `git clone https://github.com/BALA22-cyber/astar_gazeobo661.git`
- `cd ~/catkin_ws`
- `catkin build`
- `source ~/catkin_ws/devel/setup.bash`
- Try `roscd astar_gazebo661` to make sure the package can be found.


We need the Burger model for this assignment. Make sure you have the following line in `.bashrc`:
- `export TURTLEBOT3_MODEL=burger`

source the file using $ source devel/setup.bash

Start the environment by running the following command on terminal:
- `roslaunch astar_gazebo661 astar_gazebo.launch`

Dependancies
numpy
opencv
math
