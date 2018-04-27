#include "knowledge_representation/MemoryConduit.h"

using namespace std;

bool knowledge_rep::MemoryConduit::encode(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &table) {
    vector<LongTermMemoryConduit::ObjectAttribute> facings = ltmc.get_object_attribute(robot_id, "facing");
    assert(facings.size() == 1);
    int facing_id = boost::get<int>(facings.at(0).value);
    //TODO: Make sure this is table
    //assert(boost::get<std::string>(ltmc.get_object_attribute(facing_id, "concept")) == string("table"));
    vector<int> candidates = ltmc.get_objects_with_attribute_of_value("is_on", facing_id);
    stmc.resolve_object_correspondences(objects, candidates);
    // TODO: Take the ones that do not have correspondences. Make new objects for them, put them on the table
    // TODO: Forget in STMC the old objects that weren't corresponded to
    //TODO: Change sensed in ltmc on the objects
    return false;
}


vector<knowledge_rep::LongTermMemoryConduit::ObjectAttribute>
knowledge_rep::MemoryConduit::relevant_to(std::vector<int> objects) {
    vector<knowledge_rep::LongTermMemoryConduit::ObjectAttribute> obj_attrs;
    // TODO: Implement a smarter strategy for getting relevant object attributes
    // Something that will recurse and find dependencies, but also not get
    // everything in the database as a result

    // For now, use all objects
    for (const auto obj_id: ltmc.get_all_objects()) {
        auto attrs = ltmc.get_object_attributes(obj_id);
        obj_attrs.insert(obj_attrs.end(), attrs.begin(), attrs.end());
    }
    return obj_attrs;
}
