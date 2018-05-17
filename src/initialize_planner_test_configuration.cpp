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


    /*
    object(2). %concept corridor
    concept(2, corridor).
    object(3). %corridor
    is_a(3, 2).
    navigable(3).
    object(4). %concept kitchen
    concept(4, kitchen).
    object(5). %kitchen
    is_a(5, 4).
    object(6). %concept surface
    concept(6, surface).
    object(7). %concept table
    concept(7, table).
    is_a(7, 6).
    object(8). %table
    is_a(8, 7).
    navigable(8).
    is_in(8, 5).
    object(9). %concept coke
    concept(9, coke).

    at(3).

    %added by state machine
    object(10). %coke
    is_a(10, 9).
    can_be_in_concept_id(9, 4).*/

    ltmc.delete_all_objects();
    assert(ltmc.get_all_objects().size() == 1);
    int corridor_concept = ltmc.add_object();
    ltmc.add_object_attribute(corridor_concept, "concept", "corridor");
    int corridor = ltmc.add_object();
    ltmc.add_object_attribute(corridor, "is_a", corridor_concept);
    ltmc.add_object_attribute(corridor, "navigable", true);
    int kitchen_concept = ltmc.add_object();
    ltmc.add_object_attribute(kitchen_concept, "concept", "kitchen");
    int kitchen = ltmc.add_object();
    ltmc.add_object_attribute(kitchen, "is_a", kitchen_concept);
    int surface_concept = ltmc.add_object();
    ltmc.add_object_attribute(surface_concept, "concept", "surface");
    int table_concept = ltmc.add_object();
    ltmc.add_object_attribute(table_concept, "concept", "table");
    ltmc.add_object_attribute(table_concept, "is_a", surface_concept);
    int table = ltmc.add_object();
    ltmc.add_object_attribute(table, "is_a", table_concept);
    ltmc.add_object_attribute(table, "navigable", true);
    ltmc.add_object_attribute(table, "is_in", kitchen);
    int coke_concept = ltmc.add_object();
    ltmc.add_object_attribute(coke_concept, "concept", "coke");

    ltmc.add_object_attribute(corridor, "at", true);

    int coke = ltmc.add_object();
    ltmc.add_object_attribute(coke, "is_a", coke_concept);
    ltmc.add_object_attribute(coke_concept, "can_be_in_concept_id", kitchen_concept);
    
    cout << "Done!" << endl;
} 

