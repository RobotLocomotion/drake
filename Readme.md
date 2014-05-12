MeshUtil
========

Some useful tools for playing with 3D meshes in MATLAB.

SketchMesh: Generate a pretty wireframe from a model
----------------------------------------------------
Example usage:

	>> [vertices, faces] = meshutil.toolbox_graph.read_mesh('/home/rdeits/drc/software/drake/examples/Glider/meshes/GliderFuselage.obj');
	>> meshutil.sketch_mesh(vertices, faces);

![](https://rdeits.github.io/meshutil/fig/glider_sketch.png)
