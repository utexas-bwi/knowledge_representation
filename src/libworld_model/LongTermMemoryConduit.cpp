#include <villa_world_model/LongTermMemoryConduit.h>
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
using namespace ::mysqlx;
using namespace std;

namespace knowledge_rep {

	LongTermMemoryConduit::LongTermMemoryConduit(const std::string &addr, const int port, const std::string &usr,
												 const std::string &password, const std::string &db_name) {

		try {
			SessionSettings from_options(addr, port, usr, password, db_name);
			cout << "Creating session from_options: " << endl;
			sess = std::unique_ptr<Session>(new Session(from_options));
			cout << "Session accepted, creating schema." << endl;
			db = std::unique_ptr<Schema>(new Schema(*sess, db_name));
		}
		catch (const mysqlx::Error &err) {
			cout << "ERROR: " << err << endl;
			return;
		}
		catch (std::exception &ex) {
			cout << "STD EXCEPTION: " << ex.what() << endl;
			return;
		}
		catch (const char *ex) {
			cout << "EXCEPTION: " << ex << endl;
			return;
		}


	}


	LongTermMemoryConduit::~LongTermMemoryConduit() = default;

	int LongTermMemoryConduit::add_object() {

		/*Note: This method acts also as an 'add object relation' method.
		 * This means, if there is an object that already exists in the database and you want to add a relation
		 * such as 'is_on' another object, this method can be used to add just the relation to the
		 * proper object_attributes table.
		 *
		 * Also, it is implicit that attribute_name is one of {'concept', 'is_a', 'is_on', 'is_in'.}
		 * Don't try to use something else. */

		// First, the actual object ID must be added to the 'objects' table if it is a completely new object.
		Table objects = db->getTable("objects");

		Result result = objects.insert("object_id").values(NULL).execute();
        cout << result.getAffectedItemsCount() << endl;
		return result.getAutoIncrementValue();


	}


	bool LongTermMemoryConduit::delete_object(int id) {

		// TODO: Handle failure
		// TODO: Recursively remove objects that are members of directional relations
		// First, removing all references to the object
		Table table = db->getTable("object_attributes");
		TableRemove remover = table.remove();

		remover.where("attribute_value_object_id=:id").bind("id", id);
		remover.execute();

		// Next, Removing entry from the original objects table	
		remover.where("object_id=:id").bind("id", id);
		remover.execute();

		// Finally, removing entry from object_attributes table
		table = db->getTable("objects");
		remover = table.remove();
		remover.where("object_id=:id").bind("id", id);
		remover.execute();
	}

	bool LongTermMemoryConduit::object_exists(int id) {
		Table objects = db->getTable("objects");
		auto result = objects.select("object_id").where("object_id = :id").bind("id", id).execute();
		return result.count() == 1;
	}

    bool LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name,
                                                     const float float_val) {
        return false;
    }

    bool
    LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name, const bool bool_val) {
        return false;
    }

    bool LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name,
                                                     const int other_object_id) {
        return false;
    }

    bool LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name, const std::string &string_val) {

		Table object_attributes = db->getTable("object_attributes");

        TableInsert inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_string");
        inserter.values(object_id, attribute_name, string_val);

		inserter.execute();
	}


}