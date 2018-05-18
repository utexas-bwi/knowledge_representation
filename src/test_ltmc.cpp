#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>


using namespace std;


int main(int argc, const char *argv[]) {
    knowledge_rep::LongTermMemoryConduit ltmc("127.0.0.1", 33060, "root", "", "villa_krr");

    ltmc.delete_all_entities();
    // Robot should exist
    assert(ltmc.entity_exists(1));

    int coke = ltmc.add_entity();
    assert(ltmc.entity_exists(coke));

    // Get concept returns the one true concept id
    int soda = ltmc.get_concept("soda");
    assert(soda == ltmc.get_concept("soda"));

    // Get concept should return high number ids because it creates new entities and auto increments the key
    assert(coke < ltmc.get_concept("never seen before"));

    int drinkable = ltmc.get_concept("drinkable");
    int can = ltmc.add_entity();


    ltmc.add_entity_attribute(soda, "is_a", drinkable);
    ltmc.add_entity_attribute(can, "is_a", "navigable");
    ltmc.add_entity_attribute(can, "is_a", soda);
    assert(ltmc.add_entity_attribute(can, "not a real attribute", coke) == false);

    // Concept is an attribute, so we should be able to poke at it through here
    auto attributes = ltmc.get_entity_attribute(soda, "concept");
    auto at = attributes[0];
    assert(boost::get<std::string>(at.value) == string("soda"));
    auto attrs = ltmc.get_entity_attribute(can, "is_a");
    assert(attrs.size() == 2);

    // The concrete object should vanish...
    ltmc.delete_entity(can);
    assert(!ltmc.entity_exists(can));

    // Along with its attributes
    attrs = ltmc.get_entity_attributes(can);
    assert(attrs.empty());

    auto all_objs = ltmc.get_all_entities();

    ltmc.delete_all_entities();
    all_objs = ltmc.get_all_entities();
    // The robot itself is always in the knowledge base
    assert(all_objs.size() == 2);
    assert(all_objs.at(0) == 1);

    cout << "Done!" << endl;
} 

