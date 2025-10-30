import pydrake.geometry as mut  # ruff: isort: skip
import pydrake.geometry._testing as mut_testing  # ruff: isort: skip

import unittest

from pydrake.common import FindResourceOrThrow


class TestGeometryHydro(unittest.TestCase):
    def test_hydro_proximity_properties(self):
        """
        Tests the utility functions (related to hydroelastic contact) for
        setting values in ProximityProperties (as defined in
        proximity_properties.h).
        """
        props = mut.ProximityProperties()
        res_hint = 0.175
        E = 1e8
        mut.AddRigidHydroelasticProperties(
            resolution_hint=res_hint, properties=props
        )
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(
            props.GetProperty("hydroelastic", "resolution_hint"), res_hint
        )

        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertFalse(props.HasProperty("hydroelastic", "resolution_hint"))

        props = mut.ProximityProperties()
        res_hint = 0.275
        mut.AddCompliantHydroelasticProperties(
            resolution_hint=res_hint, hydroelastic_modulus=E, properties=props
        )
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(
            props.GetProperty("hydroelastic", "resolution_hint"), res_hint
        )
        self.assertTrue(
            props.HasProperty("hydroelastic", "hydroelastic_modulus")
        )
        self.assertEqual(
            props.GetProperty("hydroelastic", "hydroelastic_modulus"), E
        )

        props = mut.ProximityProperties()
        slab_thickness = 0.275
        mut.AddCompliantHydroelasticPropertiesForHalfSpace(
            slab_thickness=slab_thickness,
            hydroelastic_modulus=E,
            properties=props,
        )
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "slab_thickness"))
        self.assertEqual(
            props.GetProperty("hydroelastic", "slab_thickness"), slab_thickness
        )
        self.assertTrue(
            props.HasProperty("hydroelastic", "hydroelastic_modulus")
        )
        self.assertEqual(
            props.GetProperty("hydroelastic", "hydroelastic_modulus"), E
        )

    def test_make_convex_hull(self):
        mesh_path = FindResourceOrThrow("drake/geometry/test/quad_cube.obj")
        hull = mut._MakeConvexHull(mut.Convex(mesh_path))
        self.assertIsInstance(hull, mut.PolygonSurfaceMesh)
