#!/usr/bin/env python
import sys
import unittest
from knowledge_representation.map_loader import load_map_from_yaml


class TestLoaders(unittest.TestCase):

    def test_add_entity_works(self):
        name, annotations = load_map_from_yaml("resources/map/map.yaml")


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun("knowledge_representation", 'test_bare_bones', TestLoaders)
