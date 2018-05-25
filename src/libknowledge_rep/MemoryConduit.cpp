#include "knowledge_representation/MemoryConduit.h"
#include <tf/transform_listener.h>
#include <bwi_perception/filter.h>
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool knowledge_rep::MemoryConduit::encode(std::vector<PointCloudT::Ptr> &entities,
                                          PointCloudT::Ptr &table) {
    vector<LongTermMemoryConduit::EntityAttribute> facings = ltmc.get_entity_attributes(robot_id, "facing");
    assert(facings.size() == 1);
    int facing_id = boost::get<int>(facings.at(0).value);
    //TODO: Make sure this is table
    //assert(boost::get<std::string>(ltmc.get_entity_attribute(facing_id, "concept")) == string("table"));
    vector<int> candidates = ltmc.get_entities_with_attribute_of_value("is_on", facing_id);

    // TODO: Take the ones that do not have correspondences. Make new entities for them, put them on the table
    // TODO: Forget in STMC the old entities that weren't corresponded to
    //TODO: Change sensed in ltmc on the entities
    return false;
}


vector<knowledge_rep::LongTermMemoryConduit::EntityAttribute>
knowledge_rep::MemoryConduit::relevant_to(std::vector<int> entities) {
    vector<knowledge_rep::LongTermMemoryConduit::EntityAttribute> obj_attrs;
    // TODO: Implement a smarter strategy for getting relevant entity attributes
    // Something that will recurse and find dependencies, but also not get
    // everything in the database as a result

    // For now, use all entities
    for (const auto obj_id: ltmc.get_all_entities()) {
        auto attrs = ltmc.get_entity_attributes(obj_id);
        obj_attrs.insert(obj_attrs.end(), attrs.begin(), attrs.end());
    }
    return obj_attrs;
}

PointCloudT::Ptr knowledge_rep::MemoryConduit::get_facing_cloud(bool include_ground) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok()) {
        try {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            break;
        } catch (tf::TransformException &ex) {
            ROS_ERROR_THROTTLE(1, "%s", ex.what());
            ROS_WARN_THROTTLE(1, "Waiting for transform from base_link to map");
        }
    }


    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    double yaw = tf::getYaw(q);

    villa_octomap_server::GetPointCloudRequest req;
    villa_octomap_server::GetPointCloudResponse res;
    //req.crop_region = true;

    get_octomap_service.waitForExistence(ros::Duration(10));
    PointCloudT::Ptr from_octomap(new PointCloudT());
    if (!get_octomap_service.call(req, res)) {
        cerr << "Failed to fetch octomap" << endl;
        assert(false);
        from_octomap->clear();
        return from_octomap;
    }
    pcl::fromROSMsg(res.cloud, *from_octomap);
    PointCloudT::Ptr result(new PointCloudT());
    bwi_perception::filter_points_between<PointT>(from_octomap, result, {x, y}, yaw - M_PI / 4, yaw + M_PI / 4);
    if (!include_ground) {
        pcl::PassThrough<PointT> ground_filter;
        ground_filter.setInputCloud(result);
        ground_filter.setFilterFieldName("z");
        ground_filter.setFilterLimits(0.1, std::numeric_limits<float>::max());
        ground_filter.filter(*result);
    }
    return result;
}

sensor_msgs::PointCloud2 knowledge_rep::MemoryConduit::get_facing_cloud_ros(bool include_ground) {
    PointCloudT::Ptr cloud = get_facing_cloud(include_ground);
    sensor_msgs::PointCloud2 result;
    pcl::toROSMsg(*cloud, result);
    return result;
}