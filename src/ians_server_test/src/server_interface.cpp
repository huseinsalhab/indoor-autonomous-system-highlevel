#include <mosquitto.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define queue_length 32 

/* AUTHOR: Kyle Ebding

   This program runs a ROS node server-to-robot_mqtt_bridge that takes
   incoming MQTT messages and publishes them to the corresponding ROS
   topic.

   This program uses the Mosquitto MQTT broker to receive MQTT messages.

   COMPILING: make sure to include the argument "-l mosquitto" to include the
   Mosquitto library.
*/

// this function verifies that the connection was established successfully
void connect_callback(struct mosquitto *mosq, void *obj, int result) {
    printf("connect callback\n");
}

// this function defines how the program reacts to incoming messages
void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
    {
    bool match = 0;

    // *obj is a ROS Pub. 
    ros::Publisher* blah = ((ros::Publisher*) obj);

    fprintf(stdout, "received message '%.*s' for topic '%s'\n", message->payloadlen,
            (char*)message->payload, message->topic);
    std_msgs::String msg;   // a message object
    std::stringstream ss;
    ss << (char*)message->payload;
    // msg.data = ss.str();    // set the payload as the message data
    msg.data = (char*)message->payload;    // set the payload as the message data
    std::cout << " -- " << msg.data << " -- ";
    blah->publish(msg);
    printf("line 37 \n");
    return;

    /*
    mosquitto_topic_matches_sub("/robot/killSwitch", message->topic, &match);
    if(match)
        killSwitch_pub.pub(msg);

    mosquitto_topic_matches_sub("/robot/dst", message->topic, &match);
    if(match)
        dst_pub.pub(msg);
    mosquitto_topic_matches_sub("/robot/server_map_msgs", message->topic, &match);
    if(match)
        map_pub(msg);
    */

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "server_interface_node");
    ros::NodeHandle n;
    ros::Publisher killSwitch_pub = n.advertise<std_msgs::String>("robot/killSwitch", queue_length);
    ros::Publisher dst_pub = n.advertise<std_msgs::String>("robot/dst", queue_length);
    ros::Publisher map_pub = n.advertise<std_msgs::String>("robot/server_map_msgs", queue_length);
    
    mosquitto_lib_init(); 
    struct mosquitto *mosq = NULL;
    mosq = mosquitto_new("foo", false, (void*) &map_pub);

    if (!mosq) { // make sure the structure initialized
        printf("failed to initialize mosquitto instance \n");
        return(-1);
    }

    mosquitto_connect_callback_set(mosq, connect_callback);
    mosquitto_message_callback_set(mosq, message_callback);

    // Mosquttio Struct, Server IP, Port, Keep Alive Timer (seconds)
    if(mosquitto_connect(mosq, "35.229.88.91", 1883, 60)) {
        printf("error connecting\n");
        return(-1);
    }

    mosquitto_subscribe(mosq, NULL, "robot/killSwitch", 0);

    // run an infinite loop that listens for messages and processes them
    mosquitto_loop_forever(mosq, -1, 1);

    // if the loop exits for some reason, clean up the workspace
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
