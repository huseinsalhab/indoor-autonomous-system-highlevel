#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>

/* This ROS node is launched by a shell script in response to receiving a new 
   destination from the server. This node takes in the command line arguments 
   x and y, which are the (x,y) coordinates of the destination. These values
   are parsed, put into a move_base_msgs::MoveBaseGoal ros struct, and the 
   struct is then sent to move_base. The node then shuts down.

*/

#define NUM_ARGS 5

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
    // parse command line arguments. assumes command was called with: 
    // "rosrun ians_control set_nav_goal x_dest y_dest x_start y_start"
    if (argc < NUM_ARGS) {
        printf("error: invalid command line arguments\n");
        exit(-1);
    }
    int x_dest, y_dest; 
    sprintf(argv[3], "%d", x_dest);
    sprintf(argv[4], "%d", y_dest);

    ros::init(argc, argv, "set_nav_goal");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    /*
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    } */
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x_dest;
    goal.target_pose.pose.position.y = y_dest;
    goal.target_pose.pose.orientation.w = 1.0;


    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    /*
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal successfully sent to move_base\n");
        exit(0);
    }
    else {
        ROS_INFO("move_base error\n");
        exit(-1);
    }
    */
    //just in case the program reaches this point, exit
    return 0;
}

void set_nav_goal(int x, int y, int w, move_base_msgs::MoveBaseGoal goal) {
}
