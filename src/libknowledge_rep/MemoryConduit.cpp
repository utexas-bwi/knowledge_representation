#include "knowledge_representation/MemoryConduit.h"

using namespace std;

bool knowledge_rep::MemoryConduit::encode(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &table) {
    vector<LongTermMemoryConduit::ConceptValue> facings = ltmc.get_object_attribute(robot_id, "facing");
    assert(facings.size() == 1);
    int facing_id = boost::get<int>(facings.at(0));
    //TODO: Make sure this is table
    //assert(boost::get<std::string>(ltmc.get_object_attribute(facing_id, "concept")) == string("table"));
    vector<int> candidates = ltmc.get_objects_with_attribute_of_value("is_on", facing_id);
    stmc.resolve_object_correspondences(objects, candidates);
    // TODO: Take the ones that do not have correspondences. Make new objects for them, put them on the table
    // TODO: Forget in STMC the old objects that weren't corresponded to
    //TODO: Change sensed in ltmc on the objects
    return false;
}
