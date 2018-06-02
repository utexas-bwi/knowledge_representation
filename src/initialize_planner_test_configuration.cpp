#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>

using ::std::cout;
using ::std::endl;
using namespace std;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_planner");
    knowledge_rep::MemoryConduit mc;
    knowledge_rep::LongTermMemoryConduit ltmc("127.0.0.1", 33060, "root", "", "villa_krr");

    int empty_handed_con = ltmc.get_concept("empty_handed");
    ltmc.add_entity_attribute(1, "is_a", empty_handed_con);

    // Add this information to the locations.xml instead
    int start_position_con = ltmc.get_concept("gpsr start position");
    int location_con = ltmc.get_concept("location");
    ltmc.add_entity_attribute(start_position_con, "is_a", location_con);
    int start_position = ltmc.add_entity();
    ltmc.add_entity_attribute(start_position, "is_a", start_position_con);
    ltmc.add_entity_attribute(start_position, "map_name", "gpsr start position");
    
    cout << "Done!" << endl;
} 

