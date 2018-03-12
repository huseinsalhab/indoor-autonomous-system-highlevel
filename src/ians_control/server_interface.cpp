#include <mosquitto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define queue_length 32 
#define cmd_len 256

/* AUTHOR: Kyle Ebding

   This program runs a ROS node server-to-robot_mqtt_bridge that takes
   incoming MQTT messages and processes them to control the robot.

   This program uses the Mosquitto MQTT broker to receive MQTT messages.

   COMPILING
   make sure to use the argument "-l mosquitto" to link the Mosquitto library
*/

//motorDisable is a global variable that controls whether the robot may move
uint8_t motorDisable;

//this function is a wrapper for system() that includes error checking
int System(const char* command) {
    if(system(command)) {
        std::cout << "error calling " << command << "\n";
        return -1;
    }
    return 0;
}

//this function handles downloading a map from the server
int downloadMap(char* mapID) {
    /* functionality for multiple maps is a future feature
    char cmd[cmd_len];
    if(strcpy(cmd, "curl ") == null)
        return -1;
    if(strcat(cmd, mapID) == null)
        return -1;
    */
    char* cmd = "curl http://35.229.88.91/v1/map.png"
    return System(cmd);
}

//this function runs a shell script that starts ROS nodes used for mapping
void startMapping() {
    System("~/catkin_ws/indoor-autonomous-system-highlevel/scripts/teleop_bringup.sh");
}


//this function runs a shell script that stops ROS nodes used for mapping
void stopMapping() {
    System("~/catkin_ws/indoor-autonomous-system-highlevel/scripts/teleop_shutdown.sh");
}

// this function verifies that the connection was established successfully
void connect_callback(struct mosquitto *mosq, void *obj, int result) {
    std::cout << "mosquitto connected\n";
    return;
}

// this function defines how the program reacts to incoming messages
void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {

    fprintf(stdout, "received message '%.*s' for topic '%s'\n", message->payloadlen,
            (char*)message->payload, message->topic);
        
    std_msgs::String msg;   // a message object
    std::stringstream ss;
    ss << (char*)message->payload;
    msg.data = ss.str();    // set the payload as the message data

    // check expected topics. if there's a match, publish to that topic
    if(strcmp("robot/motorDisable", message->topic) == 0) {
        //toggle the motorDisable
        motorDisable ^= 1;
        std::cout << "motorDisable = " << motorDisable;
    }
    if(strcmp("robot/dst", message->topic) == 0) {
        //update the destination
    }
    if(strcmp("robot/map_msgs", message->topic) == 0) {
        //download the specified map
        if(downloadMap(msg.data)) {
            std::cout << "error downloading map\n";
        } 
    }
    if(strcmp("robot/mapping", message->topic) == 0) {
        if(strcmp("robot start mapping", msg.data) == 0) {
            //start mapping
        }    
        if(strcmp("robot stop mapping", msg.data) == 0) {
            //stop mapping
        }
    }
    return;
}

int main(int argc, char **argv) {

    mosquitto_lib_init(); 
    struct mosquitto *mosq = NULL;
    void* obj;
    // create a mosquitto instance. 
    mosq = mosquitto_new("foo", false, obj);

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

    mosquitto_subscribe(mosq, NULL, "robot/motorDisable", 0);
    mosquitto_subscribe(mosq, NULL, "robot/dst", 0);
    mosquitto_subscribe(mosq, NULL, "robot/map_msgs", 0);
    mosquitto_subscribe(mosq, NULL, "robot/mapping", 0);

    // run an infinite loop that listens for messages and processes them
    mosquitto_loop_forever(mosq, -1, 1);

    // if the loop exits for some reason, clean up the workspace
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
