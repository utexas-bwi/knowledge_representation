from knowledge_representation.xml_parsers.locations import LocationParser
from knowledge_representation.xml_parsers.objects import ObjectParser
from knowledge_representation.xml_parsers.questions import QuestionParser
import random
import os
from rospkg.rospack import RosPack


def get_default_xml_kbase():
    rospack = RosPack()
    xml_files_location = os.path.abspath(os.path.join(rospack.get_path('spr_qa'), '../GPSRCmdGen/CommonFiles'))
    assert (os.path.isdir(xml_files_location))
    location_xml_filename = os.path.join(xml_files_location, 'Locations.xml')
    object_xml_filename = os.path.join(xml_files_location, 'Objects.xml')
    question_xml_filename = os.path.join(xml_files_location, 'Questions.xml')
    assert (os.path.isfile(location_xml_filename) and os.path.isfile(object_xml_filename) and os.path.isfile(question_xml_filename))
    return XMLKnowledgeBase(object_xml_filename, location_xml_filename, question_xml_filename)


class XMLKnowledgeBase(object):
    def __init__(self, object_xml_file, location_xml_file, question_xml_file):
        self.location_parser = LocationParser(location_xml_file)
        self.object_parser = ObjectParser(object_xml_file)
        self.question_parser = QuestionParser(question_xml_file)
        #self.person_parser = PersonParser()
    
    def query(self, logic, argument_tuple):
        if logic == "find_location(x)":
            return self.location_parser.where_is_located(argument_tuple[0])
        elif logic == "count(x,doors)":
            return self.location_parser.how_many_doors(argument_tuple[0])
        elif logic == "count_crowd(x)":
            #TODO: need to connect crowd questions, for now just guess zero, one,or two.
            return random.randint(0,2)
        elif logic == "count_posture(x)":
            #TODO: need to connect crowd questions, for now just guess zero, one,or two.
            return random.randint(0,2)
        elif logic == "count_wearing(x)":
            #TODO: need to connect crowd questions, for now just guess zero, one,or two.
            return random.randint(0,2)
        elif logic == "count(x,y)":
            #TODO what do we do about crowd questions?
            #Ambiguous so need to check with multiple queries and see what works
            object_query = self.object_parser.how_many_objects_in_location(argument_tuple[0], argument_tuple[1])
            location_query = self.location_parser.how_many_location_in_room(argument_tuple[0], argument_tuple[1])
            #print object_query, location_query
            if object_query is None and location_query is None:
                return None
            elif object_query is None:
                return location_query
            else:
                return object_query
        elif logic == "which_person(x,y,z)":
            #TODO need to connect to akanksha's detector
            #guess for now.
            if (random.random() < 0.5):
                return argument_tuple[1]
            else:
                return argument_tuple[2]
        elif logic=="is_person(x,y)":
            if random.random() < 0.5:
                return True
            else:
                return False
            
        elif logic == "get_object_location(x)":
            return self.object_parser.get_default_location(argument_tuple[0])
        elif logic == "count_category(x)":
            return self.object_parser.get_num_in_category(argument_tuple[0])
        elif logic == "list_objects_in(x)":
            return self.object_parser.list_objects_in_location(argument_tuple[0])
        elif logic == "find_category(x)":
            return self.object_parser.find_object_category(argument_tuple[0])
        elif logic == "same_category(x,y)":
            return self.object_parser.are_objects_same_category(argument_tuple[0], argument_tuple[1])
        elif logic == "object_color(x)":
            return self.object_parser.get_object_color(argument_tuple[0])
        elif logic == "find_object(x,y)":
            if argument_tuple[0] == "heaviest" or argument_tuple[0] == "lightest" or argument_tuple[0] == "smallest" or argument_tuple[0] == "biggest":
                return self.object_parser.find_object(argument_tuple[0], argument_tuple[1])
            else:
                return self.object_parser.find_object(argument_tuple[1], argument_tuple[0])
        elif logic == "which_object(x,y,z)":
            return self.object_parser.which_object(argument_tuple[0], argument_tuple[1], argument_tuple[2])
        elif logic == "find_location(x) or get_object_location(x)":
            location = self.location_parser.where_is_located(argument_tuple[0])
            obj = self.object_parser.get_default_location(argument_tuple[0])
            if location == []:
                return ("obj",obj)
            return ("loc", location)
        elif logic == "fetch_object(x,y)":
            return (argument_tuple[0], argument_tuple[1])
        #elif logic == "saw_person(x,y)":
        #    return self.person_parser.get_age_and_gender(argument_tuple[0], argument_tuple[1])
        #elif logic == "count(crowd, x)":
        #    return self.person_parser.count_crowd(arguments_tuple[0])