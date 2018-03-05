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
*/

struct rosPubsStruct {
    ros::Publisher killSwitch_pub;
    ros::Publisher dst_pub;
    ros::Publisher map_pub;
};


// this function verifies that the connection was established successfully
void connect_callback(struct mosquitto *mosq, void *obj, int result) {
    std::cout << "connected\n";
    return;
}

// this function defines how the program reacts to incoming messages
void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    // *obj is a ROS Pub. 
    struct rosPubsStruct* rosPubs;
    rosPubs = (struct rosPubsStruct*) obj;

    fprintf(stdout, "received message '%.*s' for topic '%s'\n", message->payloadlen,
            (char*)message->payload, message->topic);
        
    std_msgs::String msg;   // a message object
    std::stringstream ss;
    ss << (char*)message->payload;
    msg.data = ss.str();    // set the payload as the message data

    // check expected topics. if there's a match, publish to that topic
    if(strcmp("robot/killSwitch", message->topic) == 0) {
        rosPubs->killSwitch_pub.publish(msg);
    }
    if(strcmp("robot/dst", message->topic) == 0) {
        rosPubs->dst_pub.publish(msg);
    }
    if(strcmp("robot/map_msgs", message->topic) == 0) {
        rosPubs->map_pub.publish(msg);
    }
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "server_interface_node");
    ros::NodeHandle n;
    struct rosPubsStruct rosPubs;
    // create publishers for each topic and assign the struct fields to them
    rosPubs.killSwitch_pub = n.advertise<std_msgs::String>("robot/killSwitch", queue_length);
    rosPubs.dst_pub = n.advertise<std_msgs::String>("robot/dst", queue_length);
    rosPubs.map_pub = n.advertise<std_msgs::String>("robot/map_msgs", queue_length);

    mosquitto_lib_init(); 
    struct mosquitto *mosq = NULL;
    // create a mosquitto instance. 
    mosq = mosquitto_new("foo", false, (void*) &rosPubs);

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
    mosquitto_subscribe(mosq, NULL, "robot/dst", 0);
    mosquitto_subscribe(mosq, NULL, "robot/map_msgs", 0);

    // run an infinite loop that listens for messages and processes them
    mosquitto_loop_forever(mosq, -1, 1);

    // if the loop exits for some reason, clean up the workspace
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
