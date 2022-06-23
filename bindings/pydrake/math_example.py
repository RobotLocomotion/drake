"""Shows examples using math utilities.

To see this interactively:
  bazel run //bindings/pydrake:py/math_example
"""

# To be a meaningful unit test, we must render the figure somehow.
# The Agg backend (creating a PNG image) is suitably cross-platform.
# Users should feel free to use a different back-end in their own code.
import os
os.environ['MPLBACKEND'] = 'Agg'  # noqa

# Now that the environment is set up, it's safe to import matplotlib, etc.
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import webbrowser

from pydrake.math import BarycentricMesh

# If running under `bazel run`, output to cwd so the user can find it.
# If running under `bazel test`, avoid polluting the test's cwd.
for env_name in ['BUILD_WORKING_DIRECTORY', 'TEST_TMPDIR']:
    if env_name in os.environ:
        os.chdir(os.environ[env_name])

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

plt.savefig('math_example.png')
assert os.path.exists('math_example.png')

# Show the figure (but not when testing).
if 'TEST_TMPDIR' not in os.environ:
    webbrowser.open_new_tab(url='math_example.png')
