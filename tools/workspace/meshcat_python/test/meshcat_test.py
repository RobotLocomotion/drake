# -*- coding: utf-8 -*-

import unittest


class TestMeshcat(unittest.TestCase):
    def test_visualizer(self):
        import meshcat
        import numpy
        import time

        vis = meshcat.Visualizer()
        vis.url()
        vis.set_object(meshcat.geometry.Box([0.2, 0.2, 0.2]))
        for theta in numpy.linspace(0, 2 * numpy.pi, 200):
            vis.set_transform(
                meshcat.transformations.rotation_matrix(
                    theta, [0, 0, 1]))
            time.sleep(0.005)
        vis.set_object(meshcat.geometry.Box([0.1, 0.1, 0.2]))
        vis.delete()
        vis["robot"].set_object(meshcat.geometry.Box([0.15, 0.35, 0.4]))
        vis["robot"]["head"].set_object(meshcat.geometry.Box([0.2, 0.2, 0.2]))
        vis["robot"]["head"].set_transform(
            meshcat.transformations.translation_matrix([0, 0, 0.32]))
        for x in numpy.linspace(0, numpy.pi, 100):
            vis["robot"].set_transform(
                meshcat.transformations.translation_matrix(
                    [numpy.sin(x), 0, 0]))
            time.sleep(0.01)
        for x in numpy.linspace(0, 2 * numpy.pi, 100):
            vis["robot/head"].set_transform(
                meshcat.transformations.translation_matrix([0, 0, 0.32]).dot(
                    meshcat.transformations.rotation_matrix(x, [0, 0, 1])))
            time.sleep(0.01)
        vis["robot/head"].delete()
        vis["robot"].delete()
        vis["sphere"].set_object(
            meshcat.geometry.Sphere(0.1),
            meshcat.geometry.MeshLambertMaterial(
                color=0xff22dd,
                reflectivity=0.8))
        vis["sphere"].delete()
        verts = numpy.random.rand(3, 100000)
        vis["perception/pointclouds/random"].set_object(
            meshcat.geometry.PointCloud(position=verts, color=verts))
        vis["perception/pointclouds/random"].set_transform(
            meshcat.transformations.translation_matrix([0, 1, 0]))
        vis["perception"].delete()
