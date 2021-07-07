//*****************************************************************************
//***********************          EXPLORATION          ***********************
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

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <termios.h>


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL CONSTANTS ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

const int    NUM_TURTLES = 5;      // turtles number
const double TOLERANCE   = 0.001;  // linear error tolerance; once linear error
                                   // falls below this threshold, the considered
                                   // turtle stops
const double KP_LINEAR  = 0.3;
const double KP_ANGULAR = 3.0;
const int    KEY_ESC    = 27;       // ASCII code equivalence
const int    SECOND     = 1000000;  // number of microseconds in one second


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL VARIABLES ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

bool stop = false;  // Flag to stop the considered turtle (1, ... , NUM_TURTLES)
                    // when it's set to 'true'
bool check[NUM_TURTLES + 1];  // Used to check that all the turtles reach their
                              // goal poses
turtlesim::Pose currentPose[NUM_TURTLES + 1];  // current pose for i-th turtle
turtlesim::Pose goalPose[NUM_TURTLES + 1];     // goal pose for i-th turtle


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
----------------------- CALLBACK & FUNCTION DEFINITIONS -----------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCURRENTPOSE_CALLBACK:        Get i-th turtle current pose
_____________________________________________________________________________*/

void getCurrentPose_Callback(const turtlesim::Pose::ConstPtr &msg, int i)
{
    currentPose[i].x     = msg->x;
    currentPose[i].y     = msg->y;
    currentPose[i].theta = msg->theta;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCH:      Get pressed key character; non-blocking function.
                Code adapted from:

                https://answers.ros.org/question/63491/keyboard-key-pressed/
_____________________________________________________________________________*/

char getch()
{
    fd_set         set;
    struct timeval timeout;
    int            rv;
    char           buff     = 0;
    int            len      = 1;
    int            filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec  = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN]  = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0) {
        // ROS_INFO("no_key_pressed"); // DEBUG
    } else
        read(filedesc, &buff, len);

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    COMPUTEANGULARERROR:        Compute angular error between direction of the
                                turtle and direction to the desired pose
_____________________________________________________________________________*/

double computeAngularError(turtlesim::Pose current_pose,
                           turtlesim::Pose goal_pose)
{
    // Create error vector
    double E_x = goal_pose.x - current_pose.x;  // Error along X component
    double E_y = goal_pose.y - current_pose.y;  // Error along Y component

    // Compute desired angle
    double desired_theta = atan2(E_y, E_x);

    // Compute angular error
    double E_theta = desired_theta - current_pose.theta;

    return E_theta;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    COMPUTELINEARERROR:     Get linear error from the turtles perspective.
_____________________________________________________________________________*/

double computeLinearError(turtlesim::Pose current_pose,
                          turtlesim::Pose goal_pose)
{
    // Create error vector
    double E_x     = goal_pose.x - current_pose.x;  // Error along X component
    double E_y     = goal_pose.y - current_pose.y;  // Error along Y component
    double E_theta = computeAngularError(
        current_pose, goal_pose);  // Compute angle between vectors
    double E_thetax = hypot(E_x, E_y);

    return E_thetax;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    INITIALIZEARRAYTOFALSE:     Set every element of the given logical array to
                                'false'
_____________________________________________________________________________*/

void initializeArrayToFalse(bool array[], int arrayLength)
{
    for (int i = 0; i < arrayLength; i++) {
        array[i] = false;
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    CHECKARRAY:     Check every element of a given logical array is 'true'
_____________________________________________________________________________*/

bool checkArray(bool array[], int arrayLength)
{
    for (int i = 0; i < arrayLength; i++) {
        if (array[i] == false)
            return false;  // At least one element is 'false' ...
    }
    return true;  // ... otherwise they're all 'true'
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MOVETURTLES:        Set turtles velocities with a proportional controller,
                        then stop them
_____________________________________________________________________________*/

void moveTurtles(ros::Publisher turtleVel_pub[])
{
    double               linearError  = 0;
    double               angularError = 0;
    geometry_msgs::Twist cmdVel[NUM_TURTLES + 1];  // Turtles velocities

    for (int i = 1; i <= NUM_TURTLES; i++) {
        char str[30] = "cmdVel_";
        char index[4];
        sprintf(index, "%d", i);
        strcat(str, index);

        linearError  = computeLinearError(currentPose[i], goalPose[i]);
        angularError = computeAngularError(currentPose[i], goalPose[i]);
        if (fabs(linearError) > TOLERANCE) {
            if (linearError > 0)
                cmdVel[i].linear.x = KP_LINEAR * linearError;
            else
                cmdVel[i].linear.x = 0;
            cmdVel[i].angular.z = KP_ANGULAR * angularError;
        } else {
            // Stop i-th turtle
            cmdVel[i].linear.x  = 0;
            cmdVel[i].angular.z = 0;
            check[i]            = true;
        }
    }

    // Publish the controls
    for (int i = 1; i <= NUM_TURTLES; i++)
        turtleVel_pub[i].publish(cmdVel[i]);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    INIT:       Turtles target the initial positions of their respective
                "consecutive" turtles
_____________________________________________________________________________*/

void init(ros::NodeHandle nh)
{
    // Useful to give enough time to acquire the current turtles poses
    if (ros::ok() && nh.ok()) {
        usleep(2 * SECOND);
        ros::spinOnce();
        usleep(2 * SECOND);
    }

    printf("\n");
    for (int i = 1; i <= NUM_TURTLES; i++) {
        printf("Turtle %i initial position:\t(%f, %f)\n", i, currentPose[i].x,
               currentPose[i].y);
    }

    usleep(4 * SECOND);
    printf("\nMoving all %i turtles ...\n", NUM_TURTLES);

    // Turtle (i+1)-th targets Turtle i-th initial position
    for (int i = 1; i <= NUM_TURTLES; i++)
        goalPose[i % NUM_TURTLES + 1] = currentPose[i];

    initializeArrayToFalse(check, NUM_TURTLES + 1);
    check[0] = true;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    CHANGETARGETS:      Turtles target last positions of their respective
                        "consecutive" turtles
_____________________________________________________________________________*/

void changeTargets()
{
    stop = false;

    for (int i = 1; i <= NUM_TURTLES; i++)
        check[i] = false;

    // Turtle (i+1)-th targets Turtle i-th previous position
    for (int i = 1; i <= NUM_TURTLES; i++)
        goalPose[i % NUM_TURTLES + 1] = currentPose[i];

    printf("Moving again ...\n");
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MANAGEPOSITIONSREACHED:     Once all the turtles've reached their goal poses
_____________________________________________________________________________*/

void managePositionsReached()
{
    stop = true;

    printf("\n");
    printf("New positions reached.\n");

    for (int i = 1; i <= NUM_TURTLES; i++) {
        printf("Turtle %i current position:\t(%f, %f)\n", i, currentPose[i].x,
               currentPose[i].y);
    }

    usleep(3 * SECOND);
    printf("\n");
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MANAGEALLPOSITIONSREACHED:      Once each turtle is at its initial position
_____________________________________________________________________________*/

void manageAllPositionsReached()
{
    printf("\n");
    printf("Each turtle has explored every other turtle initial "
           "position! :)\n");
    usleep(5 * SECOND);
    printf("\n");
    printf("Shutting down the node ...\n");
    usleep(1 * SECOND);
    printf("\n");
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MAIN:       Da main
_____________________________________________________________________________*/

int main(int argc, char **argv)
{
    // Node creation
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;

    // Subscribers for reading turtles current poses
    ros::Subscriber currentPose_sub[NUM_TURTLES + 1];
    for (int i = 1; i <= NUM_TURTLES; i++) {
        char str[30] = "/turtle";
        char index[4];
        sprintf(index, "%d", i);
        strcat(str, index);
        strcat(str, "/pose");

        currentPose_sub[i] = nh.subscribe<turtlesim::Pose>(
            str, 5, boost::bind(getCurrentPose_Callback, _1, i));
    }

    // Publishers for reading turtles velocities
    ros::Publisher turtleVel_pub[NUM_TURTLES + 1];
    for (int i = 1; i <= NUM_TURTLES; i++) {
        char str[30] = "/turtle";
        char index[4];
        sprintf(index, "%d", i);
        strcat(str, index);
        strcat(str, "/cmd_vel");

        turtleVel_pub[i] = nh.advertise<geometry_msgs::Twist>(str, 100);
    }

    init(nh);
    int movesCounter = 0;  // It counts the moves (1, ... , NUM_TURTLES)
    int c            = 0;  // key pressed (ASCII code)

    // Node frequency
    ros::Rate loop_rate(10);  // Frequency to run loops to (10 Hz)

    // Execute until node and channel are ok, and esc key is not pressed
    while (ros::ok() && nh.ok() && c != KEY_ESC) {
        ros::spinOnce();

        if (stop == false) {
            moveTurtles(turtleVel_pub);
        } else {
            changeTargets();
        }

        // If all the turtles've reached their goal poses
        if (checkArray(check, NUM_TURTLES + 1)) {
            managePositionsReached();
            movesCounter++;  // Update the counter (till 'movesCounter <
                             // NUM_TURTLES')
        }

        // If each turtle is at its initial position
        if (movesCounter == NUM_TURTLES) {
            manageAllPositionsReached();
            ros::shutdown();
        }

        c = getch();  // read character (non-blocking)

        loop_rate.sleep();
    }

    return 0;
}
