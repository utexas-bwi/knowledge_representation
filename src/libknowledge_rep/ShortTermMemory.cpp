#include <knowledge_representation/ShortTermMemory.h>

bool knowledge_rep::ShortTermMemory::remember(int object_id, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    point_cloud_memory.emplace(object_id, *cloud);
    return true;
}

bool knowledge_rep::ShortTermMemory::forget(int object_id) {
    point_cloud_memory.erase(object_id);
    return true;
}


