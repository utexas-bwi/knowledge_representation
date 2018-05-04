#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
using namespace ::mysqlx;
using namespace std;

namespace knowledge_rep {



    LongTermMemoryConduit::LongTermMemoryConduit(const std::string &addr, const uint port, const std::string &usr,
												 const std::string &password, const std::string &db_name) {

		try {
			SessionSettings from_options(addr, port, usr, password, db_name);
			sess = std::unique_ptr<Session>(new Session(from_options));
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

    /*
     * Inserts a new object into the database. Returns the object's ID so
     * it can be manipulated with other methods.
     */
    int LongTermMemoryConduit::add_object() {
        Table objects = db->getTable("objects");
        Result result = objects.insert("object_id").values(NULL).execute();
        return result.getAutoIncrementValue();
    }

    bool LongTermMemoryConduit::add_object(int id) {
        if (object_exists(id)) {
            return false;
        }
        Table objects = db->getTable("objects");
        Result result = objects.insert("object_id").values(id).execute();
        return true;
    }


    /*
     * Deletes an object and any other objects and relations that rely on it.
     */
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
        Table object_attributes = db->getTable("object_attributes");
        TableInsert inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_float");
        inserter.values(object_id, attribute_name, float_val);
        inserter.execute();
    }

    bool
    LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name, const bool bool_val) {
        Table object_attributes = db->getTable("object_attributes");
        TableInsert inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_bool");
        inserter.values(object_id, attribute_name, bool_val);
        inserter.execute();
    }


    bool LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name,
                                                     const int other_object_id) {
        Table object_attributes = db->getTable("object_attributes");
        TableInsert inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_object_id");
        inserter.values(object_id, attribute_name, other_object_id);
        inserter.execute();
        return false;
    }

    bool LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name, const std::string &string_val) {
        Table object_attributes = db->getTable("object_attributes");
        TableInsert inserter = object_attributes.insert("object_id", "attribute_name", "attribute_value_string");
        inserter.values(object_id, attribute_name, string_val);
		inserter.execute();
	}

    bool LongTermMemoryConduit::add_object_attribute(int object_id, const std::string &attribute_name,
                                                     const char string_val[]) {
        return add_object_attribute(object_id, attribute_name, std::string(string_val));
    }

    bool LongTermMemoryConduit::remove_object_attribute(int object_id, const std::string &attribute_name) {
        Table object_attributes = db->getTable("object_attributes");
        TableRemove remover = object_attributes.remove();
        remover.where("object_id = :id and attribute_name = :name").bind("id", object_id).bind("name", attribute_name);
        remover.execute();
    }


    LongTermMemoryConduit::ConceptValue LongTermMemoryConduit::unwrap_attribute_row_value(mysqlx::Value wrapped) {
        switch (wrapped.getType()) {
            case mysqlx::Value::INT64: {
                int value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::BOOL: {
                bool value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::FLOAT: {
                float value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::STRING: {
                std::string value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::RAW: {
                // NOTE: This is the case for BIT columns. Here, we assume BIT(1)
                auto value = (bool) wrapped.getRawBytes().second;
                return ConceptValue(value);
            }
        }
    }

    LongTermMemoryConduit::ConceptValue LongTermMemoryConduit::unwrap_attribute_row(Row row) {
        // This makes sense because only one column between 2 and 6 isn't null. This is
        // how we set our schema up
        std::string attribute_name = row[1];
        for (int i = 2; i < 5; i++) {
            auto col_value = row[i];

            if (col_value.isNull()) {
                continue;
            }
            return unwrap_attribute_row_value(col_value);

        }
        int value = row[5];
        return ConceptValue((bool) value);
    }

    vector<LongTermMemoryConduit::ObjectAttribute>
    LongTermMemoryConduit::unwrap_attribute_rows(std::list<Row> rows) {
        vector<ObjectAttribute> result_map;
        for (auto &row: rows) {
            int obj_id = row[0];
            std::string attribute_name = row[1];
            ConceptValue col_value = unwrap_attribute_row(row);
            result_map.push_back(ObjectAttribute(obj_id, attribute_name, col_value));
        }
        return result_map;
    }

    vector<LongTermMemoryConduit::ObjectAttribute>
    LongTermMemoryConduit::get_object_attributes(int object_id) {
        Table object_attributes = db->getTable("object_attributes");
        RowResult result = object_attributes.select("*").where("object_id = :id").bind("id", object_id).execute();
        std::list<Row> rows = result.fetchAll();

        return unwrap_attribute_rows(rows);
    }

    std::vector<LongTermMemoryConduit::ObjectAttribute>
    LongTermMemoryConduit::get_object_attribute(int object_id, const std::string &attribute_name) {
        Table object_attributes = db->getTable("object_attributes");
        RowResult result = object_attributes.select("*").where("object_id = :id and attribute_name = :attr").bind("id",
                                                                                                                  object_id).bind(
                "attr", attribute_name).execute();

        std::list<Row> rows = result.fetchAll();
        return unwrap_attribute_rows(rows);
    }

    vector<int> LongTermMemoryConduit::get_objects_with_attribute_of_value(const std::string &attribute_name,
                                                                           const int other_object_id) {
        Table object_attributes = db->getTable("object_attributes");
        RowResult result = object_attributes.select("*").where(
                "attribute_value_object_id = :id and attribute_name = :attr").bind("id",
                                                                                   other_object_id).bind(
                "attr", attribute_name).execute();
        std::list<Row> rows = result.fetchAll();
        vector<int> return_result;
        transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) {
            return boost::get<int>(unwrap_attribute_row_value(row[0]));
        });

        return vector<int>();
    }

    std::vector<int> LongTermMemoryConduit::get_all_objects() {
        vector<int> all_obj_ids;
        Table objects = db->getTable("objects");
        RowResult rows = objects.select("*").execute();
        transform(rows.begin(), rows.end(), back_inserter(all_obj_ids), [](Row row) {
            return row[0];
        });
        return all_obj_ids;
    }

    void LongTermMemoryConduit::delete_all_objects() {
        Table table = db->getTable("object_attributes");
        TableRemove remover = table.remove();
        remover.execute();

        // Finally, removing entry from object_attributes table
        table = db->getTable("objects");
        remover = table.remove();
        remover.execute();
        add_object(1);
        assert(object_exists(1));
    }



}