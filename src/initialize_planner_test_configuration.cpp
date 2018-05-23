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


    int living_room_con = ltmc.get_concept("living room");
    int living_room = ltmc.get_entities_with_attribute_of_value("is_a", living_room_con).at(0);
    ltmc.add_entity_attribute(1, "is_located", living_room);
    
    cout << "Done!" << endl;
} 

