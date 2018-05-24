#ifndef VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H
#define VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>


#include "LongTermMemoryConduit.h"
#include "ShortTermMemoryConduit.h"
#include <ros/ros.h>
#include <villa_octomap_server/GetPointCloud.h>

namespace knowledge_rep {

    class MemoryConduit {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        LongTermMemoryConduit ltmc;
        ShortTermMemoryConduit stmc;

        static const int robot_id = 1;

        ros::NodeHandle _pnh;
        ros::ServiceClient get_octomap_service;
    public:
        explicit MemoryConduit(const std::string &ltmi_adress = "127.0.0.1") : ltmc(ltmi_adress, 33060, "root", "",
                                                                                    "villa_krr"), stmc() {
            _pnh = ros::NodeHandle("~");
            get_octomap_service = _pnh.serviceClient<villa_octomap_server::GetPointCloud>("/octomap_cloud");

        }

        bool get_object_cloud(int id, PointCloudT::Ptr &cloud_out);

        bool encode(std::vector<PointCloudT::Ptr> &objects, PointCloudT::Ptr &table);

        std::vector<LongTermMemoryConduit::EntityAttribute> relevant_to(std::vector<int> objects);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_facing_cloud(bool include_ground=false);

        sensor_msgs::PointCloud2 get_facing_cloud_ros(bool include_ground=false);
    };

}

#endif //VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H
