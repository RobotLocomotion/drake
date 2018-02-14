from __future__ import absolute_import, division, print_function

import unittest
from pydrake.math import (BarycentricMesh)
import numpy as np


class TestBarycentricMesh(unittest.TestCase):
    def testSpelling(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])
        values = np.array([[0, 1, 2, 3]])

        mesh.get_input_grid()
        self.assertTrue(mesh.get_input_size() == 2)
        self.assertTrue(mesh.get_num_mesh_points() == 4)
        self.assertTrue(mesh.get_num_interpolants() == 3)
        self.assertTrue((mesh.get_mesh_point(0) == [0., 0.]).all())
        self.assertTrue(mesh.Eval(values, (0, 1))[0] == 2)

    def testMeshValuesFrom(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])

        def mynorm(x):
            return [x.dot(x)]

        values = mesh.MeshValuesFrom(mynorm)
        self.assertTrue(values.size == 4)

    def testSurf(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])
        values = np.array([[0, 1, 2, 3]])

        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        X, Y = np.meshgrid(list(mesh.get_input_grid()[0]),
                           list(mesh.get_input_grid()[1]))
        Z = np.reshape(values, X.shape)

        ax.plot_surface(X, Y, Z)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        # plt.show()


if __name__ == '__main__':
    unittest.main()
