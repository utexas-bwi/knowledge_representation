#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <knowledge_representation/MemoryConduit.h>
#include <knowledge_representation/GetCloud.h>

knowledge_rep::MemoryConduit memoryConduit;
//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};


bool get_facing_cloud_cb(knowledge_representation::GetCloud::Request &req,
                                       knowledge_representation::GetCloud::Response &res) {
    res.cloud = memoryConduit.get_facing_cloud_ros(req.include_ground);
    return true;
}


int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "memory_conduit_proxy");

    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    ros::ServiceServer forget_service = pnh.advertiseService("get_facing_cloud", get_facing_cloud_cb);

    ros::shutdown();
}