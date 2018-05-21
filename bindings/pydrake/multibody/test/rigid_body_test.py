import os
import unittest

import numpy as np

import pydrake
from pydrake.multibody import rigid_body


class TestRigidBody(unittest.TestCase):
    def test_api(self):
        name = "test_body"
        inertia = np.eye(6)

        rigid_body = rigid_body.RigidBody()

        rigid_body.set_name(name)
        self.assertEqual(rigid_body.get_name(), name)

        # This index is going to be trash because no one
        # has set it.
        test_index = rigid_body.get_body_index()

        rigid_body.set_spatial_inertia(inertia)
        self.assertTrue(np.allclose(rigid_body.get_spatial_inertia(),
                                    inertia))

        self.assertFalse(rigid_body.has_joint())
        

    .def("has_joint", &RigidBody<double>::has_joint)
    .def("getJoint", &RigidBody<double>::getJoint,
         py::return_value_policy::reference_internal)

    .def("get_position_start_index", &RigidBody<double>::get_position_start_index)

    .def("get_visual_elements", &RigidBody<double>::get_visual_elements)
    .def("add_prismatic_joint", &RigidBody<double>::add_joint<PrismaticJoint>)
    .def("add_revolute_joint", &RigidBody<double>::add_joint<RevoluteJoint>)
    .def("AddVisualElement", &RigidBody<double>::AddVisualElement);
}


        box = shapes.Box(size=box_size)
        self.assertTrue(np.allclose(box.size, box_size))
        self.assertEqual(box.getPoints().shape, (3, 8))
        self.assertEqual(len(box.getFaces()), 12)
        self.assertEqual(len(box.getFaces()[0]), 3)

        sphere = shapes.Sphere(radius=radius)
        self.assertEqual(sphere.radius, radius)
        self.assertEqual(sphere.getPoints().shape, (3, 1))
        with self.assertRaises(RuntimeError):
            sphere.getFaces()

        cylinder = shapes.Cylinder(radius=radius, length=length)
        self.assertEqual(cylinder.radius, radius)
        self.assertEqual(cylinder.length, length)

        capsule = shapes.Capsule(radius=radius, length=length)
        self.assertEqual(capsule.radius, radius)
        self.assertEqual(capsule.length, length)

        pts = np.tile(box_size, (10, 1)).T
        mesh_points = shapes.MeshPoints(pts)
        self.assertEqual(mesh_points.getPoints().shape, (3, 10))

        obj_mesh_path = os.path.join(
            pydrake.getDrakePath(), "examples/quadrotor/quadrotor_base.obj")
        obj_mesh_uri = "box_obj"
        mesh = shapes.Mesh(uri=obj_mesh_uri, resolved_filename=obj_mesh_path)
        self.assertTrue(np.allclose(mesh.scale, [1., 1., 1.]))
        self.assertEqual(mesh.uri, obj_mesh_uri)
        self.assertEqual(mesh.resolved_filename, obj_mesh_path)

    def test_visual_element_api(self):
        material_in = [0.3, 0.4, 0.5, 0.6]
        material_in_2 = [0.6, 0.7, 0.8, 0.9]
        box = shapes.Box(size=[1., 1., 1.])
        visual_element = VisualElement(box, box, np.eye(4), material_in)
        self.assertTrue(np.allclose(visual_element.getMaterial(), material_in))
        visual_element.setMaterial(material_in_2)
        self.assertTrue(np.allclose(visual_element.getMaterial(),
                                    material_in_2))
