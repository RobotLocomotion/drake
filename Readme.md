MeshUtil
========

Some useful tools for playing with 3D meshes in MATLAB.

sketch_mesh:
------------
Generate a pretty wireframe from a model

Example usage:

	>> [vertices, faces] = meshutil.toolbox_graph.read_mesh('/home/rdeits/drc/software/drake/examples/Glider/meshes/GliderFuselage.obj');
	>> meshutil.sketch_mesh(vertices, faces);

![](https://rdeits.github.io/meshutil/img/glider_sketch.png)

iris_contact_patch:
-------------------
Generate a convex contact patch on a 3D model. Requires [IRIS for MATLAB](https://github.com/rdeits/iris-distro).

Usage:

	>> meshutil.iris_contact_patch

