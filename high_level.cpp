#include "interface.h"
#include <iostream>
#include <string>
using ::std::cout;
using ::std::endl;
using namespace std;




int main(int argc, const char* argv[]) { 
	kb_interface interface("127.0.0.1", 33060, "username_here", "password_here", "villa_krr"); 

	interface.delete_object(900); 
	interface.update_object(10, "is_a", 2, "NULL", NULL, NULL); 

	// The following chain of operations creates an object with id 900 that is a coke and exists in living room 100.
	// Then it deletes it (if you uncomment that line).
	// To view the results, make sure you are connected to your server and try: 
	//	
	// 		using villa_krr;
	// 		select * from objects;
	// 		select * from object_attributes;
	//
	// if you try this before and after you will see that the object and its relations have indeed been added.
	
	
	interface.add_object(900, "is_a", 0); 
	interface.add_object(900, "is_in", 100); 
	//interface.delete_object(900);        
	cout <<"Done!" << endl;
} 

