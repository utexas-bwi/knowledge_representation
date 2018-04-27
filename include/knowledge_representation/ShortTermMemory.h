#ifndef KNOWLEDGE_REPRESENTATION_SHORTTERMMEMORY_H
#define KNOWLEDGE_REPRESENTATION_SHORTTERMMEMORY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <map>


namespace knowledge_rep {
    class ShortTermMemory {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        std::map<int, PointCloudT> point_cloud_memory;
    public:
        bool add(int object_id, PointCloudT::Ptr cloud);

        bool remove(int object_id);

    };
}


#endif //KNOWLEDGE_REPRESENTATION_SHORTTERMMEMORY_H
