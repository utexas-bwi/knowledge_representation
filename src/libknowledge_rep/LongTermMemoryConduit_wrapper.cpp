#include <boost/python.hpp>
#include <knowledge_representation/LongTermMemoryConduit.h>

using namespace boost::python;
using namespace knowledge_rep;

BOOST_PYTHON_MODULE (_libknowledge_rep_wrapper_cpp) {
    typedef LongTermMemoryConduit LTMC;
    class_<LongTermMemoryConduit>("LongTermMemoryConduit",
                                  init<std::string, uint, std::string, std::string, std::string>())
            .def("add_entity", static_cast<int (LTMC::*)()>(&LTMC::add_entity))
            .def("add_entity_attribute", static_cast<bool (LTMC::*)(int, const std::string &,
                                                                    const std::string &)>(&LTMC::add_entity_attribute))
            .def("remove_entity_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &)>(&LTMC::remove_entity_attribute))
            .def("get_entity_attribute", static_cast<std::vector<LTMC::EntityAttribute> (LTMC::*)(int,
                                                                                                  const std::string &)>(&LTMC::get_entity_attribute))
            .def("get_entity_attributes",
                 static_cast<std::vector<LTMC::EntityAttribute> (LTMC::*)(int)>(&LTMC::get_entity_attributes))
            .def("delete_entity", static_cast<bool (LTMC::*)(int)>(&LTMC::delete_entity))
            .def("entity_exists", static_cast<bool (LTMC::*)(int)>(&LTMC::entity_exists))
            .def("delete_all_entities", static_cast<void (LTMC::*)()>(&LTMC::delete_all_entities))
            .def("add_entity_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &, const char[])>(&LTMC::add_entity_attribute))
            .def("add_entity_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &, int)>(&LTMC::add_entity_attribute))
            .def("add_entity_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &, bool)>(&LTMC::add_entity_attribute))
                    // FIXME: If we make this available to python, we can't use the int one, which is the more common
                    // anyways...
//.def("add_entity_attribute",
//                 static_cast<bool (LTMC::*)(int, const std::string &, float)>(&LTMC::add_entity_attribute))
            .def("get_all_entities", static_cast<std::vector<int> (LTMC::*)()>(&LTMC::get_all_entities))
            .def("get_concept", static_cast<int (LTMC::*)(const std::string &)>(&LTMC::get_concept));
}