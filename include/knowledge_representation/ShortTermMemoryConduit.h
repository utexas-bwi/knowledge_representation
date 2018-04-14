#ifndef VILLA_WORLD_MODEL_SHORTTERMMEMORYINTERFACE_H
#define VILLA_WORLD_MODEL_SHORTTERMMEMORYINTERFACE_H
namespace knowledge_rep {

    class ShortTermMemoryConduit {

    public:
        std::vector<int>
        resolve_object_correspondences(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects,
                                       std::vector<int> candidates) {
            // TODO: Make the ROS interface
        }
    };

}
#endif //VILLA_WORLD_MODEL_SHORTTERMMEMORYINTERFACE_H
