#ifndef INTERFACE_H
#define INTERFACE_H
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>

namespace knowledge_rep {
    class LongTermMemoryConduit {

        std::unique_ptr<mysqlx::Session> sess;
        std::unique_ptr<mysqlx::Schema> db;

    public:
        LongTermMemoryConduit(const std::string &addr, const int port, const std::string &usr,
                              const std::string &password, const std::string &db_name);

        int add_object();

        bool add_object_attribute(int object_id, const std::string &attribute_name, const std::string &string_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const float float_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const bool bool_val);

        bool add_object_attribute(int object_id, const std::string &attribute_name, const int other_object_id);


        bool delete_object(int id);

        bool object_exists(int id);

        std::string get_object_type(int id);


        ~LongTermMemoryConduit();


    };
}

#endif

