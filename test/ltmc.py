#!/usr/bin/env python
import sys
import unittest
from knowledge_representation import PyAttributeList, AttributeValueType
import knowledge_representation

ltmc = knowledge_representation.get_default_ltmc()


class TestLTMC(unittest.TestCase):

    def setUp(self):
        ltmc.delete_all_attributes()
        ltmc.delete_all_entities()

    def tearDown(self):
        ltmc.delete_all_attributes()
        ltmc.delete_all_entities()

    def test_add_entity_works(self):
        coke = ltmc.add_entity()
        self.assertTrue(coke.is_valid())
        self.assertTrue(ltmc.entity_exists(coke.entity_id))

    def test_get_concept(self):
        nsb_concept = ltmc.get_concept("never seen before")
        nsb_concept.remove_instances()
        assert len(nsb_concept.get_instances()) == 0

    def test_create_instance(self):
        nsb_concept = ltmc.get_concept("never seen before")
        instance = nsb_concept.create_instance()
        self.assertTrue(instance)

    def test_get_entities_with_attribute_of_value(self):
        nsb_concept = ltmc.get_concept("never seen before")
        instance = nsb_concept.create_instance()
        instance_list = ltmc.get_entities_with_attribute_of_value("instance_of", nsb_concept.entity_id)
        assert len(instance_list) == 1

    def test_remove_instances(self):
        nsb_concept = ltmc.get_concept("never seen before")
        nsb_concept.remove_instances()
        assert len(nsb_concept.get_instances()) == 0

    def test_select_query(self):
        result_list = PyAttributeList()
        ltmc.select_query_string("SELECT * FROM entity_attributes_str", result_list)

    def test_add_attribute(self):
        nsb_concept = ltmc.get_concept("never seen before")
        instance = nsb_concept.create_instance("nsb")
        assert instance

        result = ltmc.add_new_attribute("new attribute", AttributeValueType.str)
        assert result
        assert ltmc.attribute_exists("new attribute")
        result = instance.add_attribute_str("new attribute", "as")
        assert result


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun("knowledge_representation", 'test_bare_bones', TestLTMC)
