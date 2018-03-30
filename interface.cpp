#include "interface.h"
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
using ::std::cout;
using ::std::endl;
using namespace ::mysqlx;
using namespace std;



kb_interface::kb_interface(std::string addr, int port, std::string usr, std::string password, std::string db_name) {


		try {

				SessionSettings from_options(addr, port, usr, password, db_name);

				cout << "Creating session from_options: " << endl;
				sess = new Session(from_options);
				cout <<"Session accepted, creating schema." <<endl;
				db = new Schema(*sess, db_name);

			

		}
		catch (const mysqlx::Error &err) {
		  cout <<"ERROR: " <<err <<endl;
		  return;
		}
		catch (std::exception &ex) {
		  cout <<"STD EXCEPTION: " <<ex.what() <<endl;
		  return;
		}
		catch (const char *ex) {
		  cout <<"EXCEPTION: " <<ex <<endl;
		  return;
		}


}


kb_interface::~kb_interface() {
	delete sess;
	delete db;

}



void kb_interface::add_object(int id, std::string attribute_name, int attribute_value_object_id, std::string attribute_value_string, float attribute_value_float, bool attribute_value_bool) {

		/*Note: This method acts also as an 'add object relation' method.
		 * This means, if there is an object that already exists in the database and you want to add a relation
		 * such as 'is_on' another object, this method can be used to add just the relation to the
		 * proper object_attributes table.
		 *
		 * Also, it is implicit that attribute_name is one of {'concept', 'is_a', 'is_on', 'is_in'.}
		 * Don't try to use something else. */	

			

		// First, the actual object ID must be added to the 'objects' table if it is a completely new object.


		Table objects = db->getTable("objects");
		TableInsert inserter = objects.insert();
		RowResult result = objects.select().execute();
		bool exists = false;
		for(Row r: result) {
			if(int(r.get(0)) == id) {
				exists = true;
				break;
			}
		}
		if(!exists) {
			inserter.values(id);
			inserter.execute();
		
		
		}







		Table object_attributes = db->getTable("object_attributes");
		
		// Then, attributes must be added to object_attributes table.
		
		
		
		if(attribute_name == "concept") {

			inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_string");
			inserter.values(id, attribute_name, attribute_value_string);
		}
		else {
		
			inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_object_id");
			inserter.values(id, attribute_name, attribute_value_object_id);


		}
		inserter.execute();
}




void kb_interface::update_object(int id, std::string attribute_name, int attribute_value_object_id, std::string attribute_value_string, float attribute_value_float, bool attribute_value_bool) {

	Table object_attributes = db->getTable("object_attributes");

	TableUpdate updater = object_attributes.update().where("object_id=\"" + to_string(id) + "\"");
	updater.set("attribute_name", attribute_name);
	updater.set("attribute_value_object_id", attribute_value_object_id);
	updater.set("attribute_value_string", attribute_value_string);
	updater.set("attribute_value_float", attribute_value_float);
	updater.set("attribute_value_bool", attribute_value_bool);

	updater.execute();

	// Note: This function works fine, but if you have null values in the database, calling those will get rid of them and replace them with default values
	// e.g. 0.0 for float or 0 for bool. This will need to be fixed in the future.


}	





void kb_interface::delete_object(int id) {


		// First, removing all references to the object
		Table table = db->getTable("object_attributes");
		TableRemove remover = table.remove();

		remover.where("attribute_value_object_id=\"" + to_string(id) + "\"");
		remover.execute();



		// Next, Removing entry from the original objects table	
		remover.where("object_id=\"" + to_string(id) + "\"");
		remover.execute();



		// Finally, removing entry from object_attributes table
		table = db->getTable("objects");
		remover = table.remove();
		remover.where("object_id=\"" + to_string(id) + "\"");
		remover.execute();
}

