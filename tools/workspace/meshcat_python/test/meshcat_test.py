# -*- coding: utf-8 -*-

import unittest


class TestMeshcat(unittest.TestCase):
    def test_visualizer(self):
        import meshcat

        vis = meshcat.Visualizer()
        vis.set_object(meshcat.geometry.Box([0.2, 0.2, 0.2]))
        vis.delete()
