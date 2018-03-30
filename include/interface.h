#ifndef INTERFACE_H
#define INTERFACE_H
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
using ::std::cout;
using ::std::endl;
using namespace ::mysqlx;
using namespace std;




class kb_interface {

        Session *sess;
        Schema *db;

        public:
        kb_interface(std::string addr, int port, std::string usr, std::string password, std::string db_name);


        void add_object(int id, std::string attribute_name, int attribute_value_object_id=NULL, std::string attribute_value_string="", float attribute_value_float=NULL, bool attribute_value_bool=NULL);

        void update_object(int id, std::string attribute_name, int attribute_value_object_id=NULL, std::string attribute_value_string="", float attribute_value_float=NULL, bool attribute_value_bool=NULL);

        void delete_object(int id); 

        ~kb_interface();



};

#endif

