#ifndef VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H
#define VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>


#include "LongTermMemoryConduit.h"
#include "ShortTermMemoryConduit.h"

namespace knowledge_rep {

    class MemoryConduit {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        LongTermMemoryConduit ltmc;
        ShortTermMemoryConduit stmc;

        static const int robot_id = 1;

    public:
        explicit MemoryConduit(const std::string &ltmi_adress = "127.0.0.1") : ltmc(ltmi_adress, 33060, "root", "",
                                                                                    "villa_krr"), stmc() {

        }

        bool get_object_cloud(int id, PointCloudT::Ptr &cloud_out);

        bool encode(std::vector<PointCloudT::Ptr> &objects, PointCloudT::Ptr &table);

        std::vector<LongTermMemoryConduit::EntityAttribute> relevant_to(std::vector<int> objects);

    };

}

#endif //VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H
