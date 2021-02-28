# __Exploration__

### _Author_: Livio Bisogni
###### __&copy; 2021 Turtley & Turtles Ltd.__
___
Let’s get to turtle each other!

## Prerequisites

* [ROS](http://wiki.ros.org/ROS/Installation) - An open-source, meta-operating system for your robots. Repository tested only under ROS Kinetic, though.

## How to compile
1. Move this folder (`exploration`) in `~/catkin_ws/src` (or wherever thy ROS workspace is).
2. Launch a terminal window and navigate to the aforementioned ROS workspace, e.g.,

	```
	$ cd ~/catkin_ws/
	```
3. Build the package:

	```
	$ catkin_make
	```

## How to execute
Open the terminal and type, e.g.,

```
$ roslaunch exploration move_turtle.launch
```

Launch another terminal window and type:

```
$ roslaunch exploration exploration.launch
``` 

## How to use
4. Point 3 is repeated till each turtle has visited all other turtles' positions.

![](img/e1.png)

![](img/e2.png)

![](img/e3.png)