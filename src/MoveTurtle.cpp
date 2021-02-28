//*****************************************************************************
//***********************         MOVETURTLE.CPP        ***********************
//***********************     Author: Livio Bisogni     ***********************
//*********************** © 2021 Turtley & Turtles Ltd. ***********************
//*****************************************************************************

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                                  INSTRUCTIONS

    Please read the attached `README.md` file.
_____________________________________________________________________________*/


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
--------------------------------- HEADER FILES --------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

#include "exploration/RobotStatus.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL CONSTANTS ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/
const int    ROBOT_NUMBER  = 5;
const double FORWARD_SPEED = 0.5;


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL VARIABLES ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

// Publisher and Subscriber for the topic 'team_status'
ros::Subscriber team_status_sub;
ros::Publisher  team_status_pub;

std::map<std::string, bool> robots_state;
std::string                 robot_name;

bool teamReady = false;


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
----------------------- CALLBACK & FUNCTION DEFINITIONS -----------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    PUBLUSHREADYSTATUS:     Publish a ready status message
_____________________________________________________________________________*/

void publishReadyStatus()
{
    // Create custom message and fill in its fields
    exploration::RobotStatus status_msg;

    status_msg.header.stamp = ros::Time::now();
    status_msg.robot_id     = robot_name;
    status_msg.is_ready     = true;

    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    team_status_pub.publish(status_msg);

    ROS_INFO_STREAM("Robot published ready status " << robot_name);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    WAITFORTEAM:        Wait for all the other team members to become ready
_____________________________________________________________________________*/

void waitForTeam()
{
    ros::Rate loopRate(1);

    // Wait until all robots are ready...
    while (!teamReady) {
        ROS_INFO_STREAM("Robot waiting for team " << robot_name);
        publishReadyStatus();
        ros::spinOnce();
        loopRate.sleep();
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    TEAMSTATUSCALLBACK:     Receive a team status
_____________________________________________________________________________*/

void teamStatusCallback(const exploration::RobotStatus::ConstPtr &status_msg)
{
    if (teamReady)
        return;

    robots_state[status_msg->robot_id] = status_msg->is_ready;

    int ready_counter = 0;
    for (auto robot : robots_state) {
        if (robot.second)
            ready_counter++;
    }

    if (ready_counter == ROBOT_NUMBER) {
        ROS_INFO_STREAM("Robot: Team is ready. Let's move! " << robot_name);
        teamReady = true;
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MAIN:       Da main
_____________________________________________________________________________*/

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "move_turtle");
    ros::NodeHandle node;

    // Get the name
    robot_name = node.getNamespace();
    robot_name = robot_name.substr(1, robot_name.npos);

    // A publisher for the motion data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Publish and subscribe to 'team_status' messages
    team_status_pub =
        node.advertise<exploration::RobotStatus>("/team_status", 10);
    team_status_sub = node.subscribe("/team_status", 20, &teamStatusCallback);

    publishReadyStatus();
    waitForTeam();

    // Twist initialization
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;

    // Loop at 10Hz, publishing movement commands until we shut down
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");

    while (ros::ok()) {
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}