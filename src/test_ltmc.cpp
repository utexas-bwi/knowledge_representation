#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>


using namespace std;


int main(int argc, const char *argv[]) {
    knowledge_rep::LongTermMemoryConduit ltmc("127.0.0.1", 33060, "root", "", "villa_krr");

    assert(ltmc.object_exists(1));

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
    assert(ltmc.add_object_attribute(can, "not a real attribute", coke) == false);

    auto attributes = ltmc.get_object_attribute(soda, "concept");
    auto at = attributes[0];
    assert(boost::get<std::string>(at.value) == string("soda"));
    auto attrs = ltmc.get_object_attribute(can, "navigable");
    assert(attrs.size() == 1);
    auto attr = attrs[0];
    assert(boost::get<bool>(attr.value) == true);
    ltmc.delete_object(coke);
    assert(!ltmc.object_exists(coke));

    auto all_objs = ltmc.get_all_objects();

    ltmc.delete_all_objects();
    all_objs = ltmc.get_all_objects();
    // The robot itself is always in the knowledge base
    assert(all_objs.size() == 1);
    assert(all_objs.at(0) == 1);

    cout << "Done!" << endl;
} 

