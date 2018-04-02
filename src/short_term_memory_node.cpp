#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};


int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "short_term_memory_node");

    ros::NodeHandle n;


    ros::shutdown();
}