#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>


using namespace std;


int main(int argc, const char *argv[]) {
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


    int coke = ltmc.add_object();
    int soda = ltmc.add_object();
    int drinkable = ltmc.add_object();
    int can = ltmc.add_object();
    assert(ltmc.object_exists(coke));
    ltmc.add_object_attribute(coke, "concept", "coke");
    ltmc.add_object_attribute(soda, "concept", "soda");
    ltmc.add_object_attribute(drinkable, "concept", "drinkable");
    ltmc.add_object_attribute(coke, "is_a", soda);
    ltmc.add_object_attribute(soda, "is_a", drinkable);
    ltmc.add_object_attribute(can, "navigable", true);
    ltmc.add_object_attribute(can, "is_a", coke);

    auto attributes = ltmc.get_object_attribute(soda, "concept");
    auto at = attributes[0];
    assert(boost::get<std::string>(at.value) == string("soda"));
    auto attr = ltmc.get_object_attribute(can, "navigable")[0];
    assert(boost::get<bool>(attr.value) == true);
    ltmc.delete_object(coke);
    assert(!ltmc.object_exists(coke));

    auto all_objs = ltmc.get_all_objects();

    cout << "Done!" << endl;
} 

