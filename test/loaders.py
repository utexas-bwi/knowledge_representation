#!/usr/bin/env python
import sys
import unittest
from knowledge_representation.map_loader import load_map_from_yaml
import os

resource_path = os.path.dirname(__file__) + "/resources"


class TestLoaders(unittest.TestCase):

    def test_load_annotator_tool_map_works(self):
        name, annotations = load_map_from_yaml(resource_path + "/map/map.yaml")
        points, poses, regions = annotations
        self.assertEqual(2, len(points))
        self.assertEqual(2, len(poses))
        self.assertEqual(1, len(regions))

    def test_load_inkscape_map_works(self):
        name, annotations = load_map_from_yaml(resource_path + "/map/map_inkscape.yaml")
        points, poses, regions = annotations
        self.assertEqual(5, len(points))
        self.assertEqual(2, len(poses))
        self.assertEqual(2, len(regions))


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun("knowledge_representation", 'test_bare_bones', TestLoaders)
