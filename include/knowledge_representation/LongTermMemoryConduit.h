#ifndef INTERFACE_H
#define INTERFACE_H
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>

namespace knowledge_rep {
    class LongTermMemoryConduit {

        std::shared_ptr<mysqlx::Session> sess;
        std::shared_ptr<mysqlx::Schema> db;

    public:
        typedef boost::variant<int, float, bool, std::string> ConceptValue;

        struct ObjectAttribute {
            int object_id;
            std::string attribute_name;
            ConceptValue value;

            ObjectAttribute(int object_id, std::string attribute_name, ConceptValue value) : object_id(object_id),
                                                                                             attribute_name(std::move(
                                                                                                     attribute_name)),
                                                                                             value(value) {}

        };
        LongTermMemoryConduit(const std::string &addr, const uint port, const std::string &usr,
                              const std::string &password, const std::string &db_name);

        int add_object();

        bool add_attribute(const std::string &name);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const std::string &string_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const float float_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const bool bool_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const int other_object_id);

        bool remove_object_attribute(int object_id, const std::string &attribute_name);

        std::vector<ObjectAttribute> get_object_attributes(int object_id);

        std::vector<ObjectAttribute> get_object_attribute(int object_id, const std::string &attribute_name);

        //TODO: Provide versions for the other possible attribute value types.
        std::vector<int>
        get_objects_with_attribute_of_value(const std::string &attribute_name, const int other_object_id);

        bool delete_object(int id);

        bool object_exists(int id);

        bool delete_attribute(int id);

        bool attribute_exists(int id);

        std::string get_object_type(int id);

        ~LongTermMemoryConduit();

        void delete_all_objects();

        bool add_object_attribute(int object_id, const std::string &attribute_name, const char string_val[]);

        std::vector<int> get_all_objects();


    private:
        std::vector<ObjectAttribute> unwrap_attribute_rows(std::list<mysqlx::Row> rows);

        LongTermMemoryConduit::ConceptValue unwrap_attribute_row_value(mysqlx::Value wrapped);

        LongTermMemoryConduit::ConceptValue unwrap_attribute_row(mysqlx::Row row);

        bool add_object(int id);


    };
}

#endif

