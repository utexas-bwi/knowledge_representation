#include <villa_world_model/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <villa_world_model/MemoryConduit.h>

using ::std::cout;
using ::std::endl;
using namespace std;




int main(int argc, const char* argv[]) {
    //MemoryConduit mc();
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


    int new_obj = ltmc.add_object();
    assert(ltmc.object_exists(new_obj));
    ltmc.add_object_attribute(new_obj, "concept", "coke");


	cout <<"Done!" << endl;
} 

