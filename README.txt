READ ME:
AURO INDIVIDUAL ASSESSMENT:
- Go to solution > solution to find ros nodes and controllers
- Project report found in report section

HOW TO RUN FILES:

Ensure no files/folders in your home directory are named auro_ws
If so, please rename them accordingly.
Open the Xfce Terminal and create a new workspace directory:

mkdir ~/auro_ws

Transfer the src folder from the submission folder to the new workspace directory: auro_ws

Run the command to build the packages:

colcon build --symlink-install && source install/local_setup.bash

Remember that you will either need to run the following command every time you start a new terminal, unless you add it to your ~/.bashrc file:

source ~/auro_ws/install/local_setup.bash


Before you can run the example code, you will need to set an environment variable to tell Gazebo where to find the TurtleBot3 models. You will need to run this command every time you start a new terminal, or you can add it to the end of your ~/.bashrc file.

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/york/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/

You may also need to set the GAZEBO_PLUGIN_PATH environment variable to use the custom differential drive plugin, for example:

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/auro_ws/build


Once the package has been build successfully run the following command to run the code:

ros2 launch solution solution_nav2_launch.py

If for any reason gazebo does not start correctly kill all rogue processes using the following and then rerun the command above:

killall -9 gazebo & killall -9 gzserver & killall -9 gzclient

