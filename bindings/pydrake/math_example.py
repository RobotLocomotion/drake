"""Shows examples using math utilities."""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from pydrake.math import BarycentricMesh

# Plot a surface using BarycentricMesh.
mesh = BarycentricMesh([{0, 1}, {0, 1}])
values = np.array([[0, 1, 2, 3]])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

X, Y = np.meshgrid(list(mesh.get_input_grid()[0]),
                   list(mesh.get_input_grid()[1]))
Z = np.reshape(values, X.shape)

ax.plot_surface(X, Y, Z)
ax.set_xlabel('x')
ax.set_ylabel('y')
plt.show()
