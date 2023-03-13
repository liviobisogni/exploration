# Exploration

A ROS package that enables multiple robots to communicate and move toward each other's positions.

<p align="center">
	<a href="#prerequisite">Prerequisite</a>
	<span> • </span>
	<a href="#compile">Compile</a>
	<span> • </span>
	<a href="#execute">Execute</a>
	<span> • </span>
	<a href="#use">Use</a>
	<span> • </span>
	<a href="#screenshots">Screenshots</a>
</p>

## <a id="prerequisite"></a>Prerequisite

* [ROS](http://wiki.ros.org/ROS/Installation) - An open-source, meta-operating system for your robots. The repository has been tested using ROS Kinetic.


## <a id="compile"></a>How to Compile
1. Move this package folder (`exploration`) to the `src` directory of your ROS workspace, for example `~/catkin_ws/src/`.

2. Open a terminal window and navigate to your ROS workspace directory, e.g.:

	```bash
	cd ~/catkin_ws/
	```
3. Build the package using the `catkin_make` command:

	```bash
	catkin_make
	```
This will compile the package and generate the necessary files for running the ROS nodes.


## <a id="execute"></a>How to Execute
1. Open a terminal window.

2. Navigate to your ROS workspace directory.

3. Launch the package using the following command:

	```bash
	roslaunch exploration move_turtle.launch
	```
4. Launch another terminal window.
5. Navigate to your ROS workspace directory.
6. Launch the package using the following command:

	```bash
	roslaunch exploration exploration.launch
	``` 
This will launch the ROS nodes required to run the package.


## <a id="use"></a>How to Use

1. Adjust the `x`, `y`, and `theta` variables in `consensus.launch` (starting from line 13) to set the initial positions of the additional robots (the first robot is spawned at the center of the workspace). Ensure that `NUM_TURTLES` in `consensus.cpp` and `ROBOT_NUMBER` in `moveTurtle.cpp` are updated to reflect the total number of robots, including the first robot and any additional robots.
2. Each robot will communicate its position with the others.
3. Then, they will start moving toward the positions of the consecutive robot.
4. Step 3 will be repeated until each robot has visited all other robots' positions.
5. Finally, they will return to their initial positions.
6. Throughout the process, various types of information will be printed on the terminal. Wait until the exploration node is shut down or press `ESC` to exit the program at any time.


## <a id="screenshots"></a>Screenshots

* Initial positions:
<p align="center" width="100%">
    <img width="61.8%" src="img/e1.png"> 
</p>

* Turtles in motion:
<p align="center" width="100%">
    <img width="61.8%" src="img/e2.png"> 
</p>

* Returning to their initial positions (after exploring all other positions):
<p align="center" width="100%">
    <img width="61.8%" src="img/e3.png"> 
</p>