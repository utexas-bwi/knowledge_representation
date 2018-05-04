#include <boost/python.hpp>
#include <knowledge_representation/LongTermMemoryConduit.h>

using namespace boost::python;
using namespace knowledge_rep;

BOOST_PYTHON_MODULE (_libknowledge_rep_wrapper_cpp) {
    typedef LongTermMemoryConduit LTMC;
    class_<LongTermMemoryConduit>("LongTermMemoryConduit",
                                  init<std::string, uint, std::string, std::string, std::string>())
            .def("add_object", static_cast<int (LTMC::*)()>(&LTMC::add_object))
            .def("add_object_attribute", static_cast<bool (LTMC::*)(int, const std::string &,
                                                                    const std::string &)>(&LTMC::add_object_attribute))
            .def("remove_object_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &)>(&LTMC::remove_object_attribute))
            .def("get_object_attribute", static_cast<std::vector<LTMC::ObjectAttribute> (LTMC::*)(int,
                                                                                                  const std::string &)>(&LTMC::get_object_attribute))
            .def("get_object_attributes",
                 static_cast<std::vector<LTMC::ObjectAttribute> (LTMC::*)(int)>(&LTMC::get_object_attributes))
            .def("delete_object", static_cast<bool (LTMC::*)(int)>(&LTMC::delete_object))
            .def("object_exists", static_cast<bool (LTMC::*)(int)>(&LTMC::object_exists))
            .def("delete_all_objects", static_cast<void (LTMC::*)()>(&LTMC::delete_all_objects))
            .def("add_object_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &, const char[])>(&LTMC::add_object_attribute))
            .def("get_all_objects", static_cast<std::vector<int> (LTMC::*)()>(&LTMC::get_all_objects));
}