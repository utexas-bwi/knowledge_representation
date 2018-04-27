#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>

using ::std::cout;
using ::std::endl;
using namespace std;


int main(int argc, const char *argv[]) {
    knowledge_rep::MemoryConduit mc;
    knowledge_rep::LongTermMemoryConduit ltmc("127.0.0.1", 33060, "root", "", "villa_krr");

    // The following chain of operations creates an object with id 900 that is a coke and exists in living room 100.
    // Then it deletes it (if you uncomment that line).
    // To view the results, make sure you are connected to your server and try:
    //
    // 		using villa_krr;
    // 		select * from objects;
    // 		select * from object_attributes;
    //
    // if you try this before and after you will see that the object and its relations have indeed been added.

    /*
     *
     * object(1).
graspable(1).
object(2).
surface(2).
is_on(1, 2, 0).
navigable(2).
person(3).
object(4).
navigable(4).
is_near(3, 4).

hand_empty(0).
at(2, 0).

     */
    ltmc.delete_all_objects();
    assert(ltmc.get_all_objects().size() == 1);
    int coke = ltmc.add_object();
    ltmc.add_object_attribute(coke, "graspable", true);
    int table = ltmc.add_object();
    ltmc.add_object_attribute(table, "surface", true);
    ltmc.add_object_attribute(table, "navigable", true);
    ltmc.add_object_attribute(coke, "is_on", table);
    int person = ltmc.add_object();
    ltmc.add_object_attribute(person, "person", true);
    int bed = ltmc.add_object();
    ltmc.add_object_attribute(bed, "navigable", true);
    ltmc.add_object_attribute(person, "is_near", bed);
    //bool success = mc.encode_internal_state();
    //assert(success);
    cout << "Done!" << endl;
} 

