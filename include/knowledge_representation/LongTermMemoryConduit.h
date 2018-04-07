#ifndef INTERFACE_H
#define INTERFACE_H
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>

namespace knowledge_rep {
    class LongTermMemoryConduit {

        std::unique_ptr<mysqlx::Session> sess;
        std::unique_ptr<mysqlx::Schema> db;

    public:
        LongTermMemoryConduit(const std::string &addr, const uint port, const std::string &usr,
                              const std::string &password, const std::string &db_name);

        int add_object();

        bool add_object_attribute(int object_id, const std::string &attribute_name, const std::string &string_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const float float_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const bool bool_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const int other_object_id);

        bool remove_object_attribute(int object_id, const std::string &attribute_name);

        std::multimap<std::string, boost::variant<int, float, bool, std::string> >  get_object_attributes(int object_id);
        std::vector<boost::variant<int, float, bool, std::string> >  get_object_attribute(int object_id, const std::string &attribute_name);


        bool delete_object(int id);

        bool object_exists(int id);

        std::string get_object_type(int id);


        ~LongTermMemoryConduit();


        bool add_object_attribute(int object_id, const std::string &attribute_name, const char string_val[]);
    };
}

#endif

