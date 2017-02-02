
.. _models:

***************
Models in Drake
***************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _models_intro:

Introduction
============

This document discusses how physical objects (i.e. robots and other
objects such as furniture) are represented in Drake .

The representations of physical objects are modelled in Drake declaratively
with either URDF (Unified Robot Description Format) or SDF, which are both
XML formats.

* `SDF Reference <http://sdformat.org/>`_

* `URDF Reference <urdf/drakeURDF.html#://>`_

The target audience for this document is for those who want to model a physical
object in Drake.  You should have some understanding of XML formats, and be
able to run Drake. However, you don't need to know C++. This document is not
meant as a tutorial of creating URDF/SDF files, which you can find elsewhere.

In this document, we call the URDF or SDF representation of the object a
model. The model of an object will describe the object's physical properties,
 for example, visual properties to describe the appearance of an object. The
 object's physical form can be approximated by geometric primitives, such as
 cylinders or spheres, or by a more-detailed triangular mesh.

Other physical properties that can be described in an URDF/SDF file include
dynamic properties (how the object moves), with the parameters mass and
inertia. Contact properties (how the object responds to contacting other
objects) have collision parameters.

These properties allow you to do simulation, visualization, motion planning,
and control.

.. _terminology:

Terminology
===========

`Mesh <https://en.wikipedia.org/wiki/Polygon_mesh>`_  - a collection of
vertices, edges, and faces that describe the shape of a 3D object. A mesh
often contains hundreds of vertices to describe a complex shape.  In Drake,
meshes must be triangulated. Furthermore, for dynamic objects, meshes will be
 replaced with the convex hulls of those meshes.

`SDF <http://sdformat.org/>`_ - Simulation Description Format. An XML format
that describes objects and environments for robot simulators, visualization,
and control. SDF is OSRF's successor to URDF. Relative to URDF, SDF is being
more actively maintained.

`Texture <https://en.wikipedia.org/wiki/Texture_mapping#Texture_maps>`_  -
the digital representation of the surface of an object. May include color,
brightness, transparency and reflectivity (shininess). The texture is mapped
onto an already available surface.

`URDF <http://wiki.ros.org/urdf/XML>`_ - Unified Robot Description Format, an
XML format for representing one robot model (but not everything else in the
world, or the pose of the robot in the world: see SDF).  Drake has its own
`version of URDF <urdf/drakeURDF.html#://>`_  with additional custom tags.

.. _model_file_formats:

File formats for Drake model components
=======================================

Model file format
-----------------

Drake's models can be defined in either URDF or SDF file formats, but SDF is
newer and so is preferred. They are similar formats. Both are XML-based.

Mesh file format
----------------
Drake can accept the following file formats for meshes:

- `DAE <https://en.wikipedia.org/wiki/COLLADA>`_ (digital asset exchange) - can
  include textures.

- `OBJ <https://en.wikipedia.org/wiki/Wavefront_.obj_file>`_ - a 3D geometry
  definition file format.

- `STL <https://en.wikipedia.org/wiki/STL_(file_format)>`_ (\ **ST**\ ereo
  **L**\ithography) - describes only the surface geometry
  of a three-dimensional object without any representation of color,
  texture or other common CAD model attributes.

- `WRL <https://en.wikipedia.org/wiki/VRML>`_- obsolete, but used in Drake. A
  text file format. Allows you to specify surface color, textures, shininess,
  transparency, and other parameters. Vertices and edges for a 3D polygon can
  be specified.

.. _model_contents:

The content of a Model file
===========================

For a simple example, see `this cinder block example <https://github
.com/RobotLocomotion/drake/blob/master/drake/examples/Atlas
/sdf/cinder_block_2/model-1_4.sdf>`_ . As you can see, an SDF file is an XML
file.

The way the model looks in an SDF file is given by the "visual" tag. In the
example, it looks like this::

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://cinder_block_2/meshes/cinder_block.dae</uri>
          </mesh>
        </geometry>
      </visual>

The visual is defined to be a mesh given by `this DAE file <https://github
.com/RobotLocomotion/drake/blob/master/drake/examples/Atlas/sdf
/cinder_block_2/meshes/cinder_block.dae>`_.  The DAE mesh file defines many
things concerning the look of the object, including a reference to a PNG file
which defines the texture. If you open the PNG file in an image viewing program
you will see it shows the pitted texture and grey color of a cinder block.

While the visual properties define how the model looks, often we need to
describe the shape of the hard parts of the model (the rigid bodies), which
the SDF file defines with a "collision" tag. You may wonder why we don't use
the visual mesh for the collision element.  That is possible, and it would be
accurate, but a mesh file is usually very detailed, often containing
thousands of vertices. The vertices are used to determine if one object is
colliding with another object, so the more vertices, the more computation.

If you look at the cinder block SDF file, you'll see that the geometries of
the collision elements are defined as boxes, instead of using the mesh. In
this cinder block, the outline of the object is a rectangle (ignoring the
holes in the cinder block), so we can use the box shape.  Other simple
geometric shapes that you can use in the collision element include cylinder
and sphere.

See the `URDF <urdf/drakeURDF.html#://>`_ or `SDF <http://sdformat.org/>`_
references for more information.

The best way to learn how to create a model is to examine and change an
existing, working model.  `This tutorial <https://www.youtube
.com/watch?v=gugV8IMyHnY>`_ walks through editing a model in Drake.

.. _simplifying_meshes:

Simplifying a Mesh
==================

Meshes of complicated objects contain many vertices. The detail of the mesh
makes the visual representation look realistic. However when a mesh is used
for a collision element, collision software has to process the entire mesh,
which can be slow. For these reasons it is often desirable for the collision
model to use a simpler form than the original high density mesh.

Q: What is the easiest way to use something simpler for a collision model?

A: If you are starting with a mesh of an object (perhaps from a 3D scanner),
you may be able to use a primitive geometric shape (e.g., cylinder, box, or
sphere) instead of the mesh, as described above. To do this, consider a tool
like `Gazebo <http://gazebosim.org/>`_, which provides a visual editor of model
files, and allows you to modify primitive geometric shapes in your model.

Q: A single geometric shape is not accurate enough for my needs. What else
can I do?

A: You can use multiple overlapping geometric shapes to define slightly more
complex collision elements.

Q: I need something much more accurate than these simple shapes. How do I
simplify a mesh?

A: Sometimes you really need the accuracy of a mesh.

In the `iiwa14.urdf robotic arm example <https://github
.com/RobotLocomotion/drake/blob/master/drake/examples/kuka_iiwa_arm/urdf
/iiwa14.urdf>`_, we use meshes for the visual geometry elements, and
cylinders for the collision elements, except for the last links where meshes
are used.  The idea is that for links of the arm, we don't need exact shapes
for collision, where we don't expect to need precision. Even if we were to
use an "elbow" to shove an object out of the way, we probably don't need
exact accuracy. It's possible we may desire more precision for the links at
the end of the arm, if the arm is reaching into tight or crowded spaces.

An existing complex mesh can be simplified using a tool like `Blender
<https://www.blender.org/>`_, a free and open source 3D creation suite.

