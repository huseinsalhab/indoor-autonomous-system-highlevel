#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <stdio.h>

/* This ROS node subscribes to the topic to list for updates of the
   destination from the server. This node takes in the command line arguments 
   x and y, which are the (x,y) coordinates of the destination. These values
   are parsed, put into a geometry_msgs/PoseStamped message, and the message is
   published to the move_base_simple/goal topic. 

*/

#define NUM_ARGS 5
#define PUB_TOPIC "move_base_goal/simple"
#define SUB_TOPIC "server_goal"
#define MESSAGE_QUEUE_SIZE 1
class SubAndPub {
public:
    SubAndPub() {
        pub = nh.advertise<geometry_msgs::PoseStamped>(PUB_TOPIC, MESSAGE_QUEUE_SIZE);
        sub = nh.subscribe(SUB_TOPIC, MESSAGE_QUEUE_SIZE, &SubAndPub::callback, this);
        quat.setRPY(0.0, 0.0, 0);
    }
    void callback(const geometry_msgs::PoseStamped &msg) {
        tf::Stamped<tf::Pose> pose = tf::Stamped<tf::Pose>(
                tf::Pose(quat, tf::Point(x, y, 0.0)),
                ros::Time::now(), 
                fixed_frame);
        tf::poseStampedTFToMsg(pose, goal);
        pub.publish(goal);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::string fixed_frame = "map";
    tf::Quaternion quat;
    geometry_msgs::PoseStamped goal;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "set_nav_goal");

    SubAndPub foo;
    ros::spin();

    return 0;
}

