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

This document discusses how physical objects (i.e., robots and other objects
such as furniture) are represented in Drake.

The representations of physical objects are modeled in Drake declaratively
using either URDF (Unified Robot Description Format) or SDF (Simulation
Description Format), which are both XML formats.

* `SDF Reference <http://sdformat.org/>`_

* `Drake's version of URDF <urdf/drakeURDF.html#://>`_.  Drake's URDF has
  extensions to the `official ROS URDF <http://wiki.ros.org/urdf/XML>`_.

The target audience for this document is for those who want to model a
physical object for use in Drake. You should have some understanding of XML
formats, and be able to run Drake. You don't need to know C++. This document
provides enough reference material for someone to edit or create a
representation of a simple object and its environment, by defining the
terminology and providing an overview of what's in a model file. This
document is not meant as a tutorial for creating URDF/SDF files, which you
can find `elsewhere <http://gazebosim.org/tutorials?tut=build_robot>`_. For
deeper understanding of polygonal modeling and advanced topics like meshes
and textures, we try to provide links to further information, but you may
have to do more of your own research and education.

In this document, we call the URDF or SDF representation of the object a
model. The model of an object will describe the object's physical properties,
including visual, contact, and dynamic properties. Visual properties
describe the appearance of an object, such as its shape and texture. Dynamic
properties describe how the object moves and have parameters like mass and
spatial inertia. Contact properties describe how an object behaves when in
contact with other objects and include numerous collision parameters.

These properties allow you to do simulation, visualization, motion planning,
and control.

.. _models_terminology:

Terminology
===========

`Mesh <https://en.wikipedia.org/wiki/Polygon_mesh>`_ - a polygon mesh which
is a collection of vertices, edges, and faces that describes the surface of a
3D object. For a mesh to represent the fine details of a surface, a complex
mesh is required. Processing a mesh gets more expensive as the mesh becomes
more complex. Therefore, it is important to use a mesh with just enough
detail to serve the purpose. To be compatible with Drake, all of the faces
in the mesh must be triangles. Many modeling tools have a simple tool for
"triangulating" the mesh.

`SDF <http://sdformat.org/>`_- Simulation Description Format. An XML format
that describes objects and environments for robot simulators, visualization,
and control. SDF is OSRF's successor to URDF. Unlike URDF, SDF allows
multiple objects to be modeled in a single SDF file, and allows you to model
the pose of the robot in the world. Relative to URDF, SDF is being more
actively maintained.

`Texture <https://en.wikipedia.org/wiki/Texture_mapping#Texture_maps>`_ - the
digital representation of an object's surface. A texture may include color,
brightness, transparency, reflectivity, and other aspects. The texture is
mapped onto a preexisting surface.

`URDF <urdf/drakeURDF.html#://>`_- Unified Robot Description Format, an XML
format for representing one model of a robot or other object. Unlike SDF,
only one object can be modeled per file, and pose cannot be modeled. Drake's
URDF extends the `official ROS URDF <http://wiki.ros.org/urdf/XML>`_.

.. _models_file_formats:

File formats for Drake model components
=======================================

.. _models_model_file_formats:

Model file format
-----------------

Drake's models can be defined in either URDF or SDF file formats, but SDF is
newer and thus preferred. They are similar formats. Both are XML-based.

.. _models_mesh_file_formats:

Mesh file format
----------------
Drake can process the following mesh file format (file extension):

- `OBJ <https://en.wikipedia.org/wiki/Wavefront_.obj_file>`_ - a 3D geometry
  definition file format.

If Drake processes an SDF or URDF file and finds a mesh file name extension that
is not a type that it can process (currently only OBJ), it will see if a file
exists with the same path and name but with an "obj" extension.

The following are mesh file formats that are not processed by Drake, but may be
found in the Drake repository and may be used to find OBJ versions:

- `DAE <https://en.wikipedia.org/wiki/COLLADA>`_ (digital asset exchange) - can
  include textures. An `issue exists to add Drake support for DAE files
  <https://github.com/RobotLocomotion/drake/issues/2941>`_.

- `STL <https://en.wikipedia.org/wiki/STL_(file_format)>`_ (\ **ST**\ ereo
  **L**\ithography) - describes only the surface geometry
  of a three-dimensional object without any representation of color,
  texture or other common CAD model attributes.  An
  `issue exists to add Drake support for STL files
  <https://github.com/RobotLocomotion/drake/issues/2941>`_.

- `WRL <https://en.wikipedia.org/wiki/VRML>`_- an obsolete mesh format that is
  superseded by a new format called X3D. Allows you to specify surface color,
  textures, shininess, transparency, and other parameters. Vertices and edges
  for a 3D polygon can be specified.

.. _models_contents:

The content of a Model file
===========================

The best way to learn how to create a model is to examine and change an
existing, working model.  `This tutorial <https://www.youtube
.com/watch?v=gugV8IMyHnY>`_ walks through editing a model in Drake.

For detailed information on the elements of a model, see the
`URDF <urdf/drakeURDF.html#://>`_ or `SDF <http://sdformat.org/>`_
references for more information.

.. _models_shape:

Modelling the shape of an object
--------------------------------

To model a shape for visual purposes, use URDF's or SDF's ``<visual>`` tag. To
model a shape for the purpose of determining contact between your model and
other objects, use URDF's or SDF's ``<collision>`` tag. (While both SDF and URDF
use those same tags, note that the structure and content of those tags differ.)

To explain why you might want to define the collision element differently than
the visual element, let's explore the different ways of defining shapes.

An object's shape can be modelled using a 3D scanner, which produces a polygon
mesh. Meshes of geometrically complicated objects contain many polygons. The
detail (density) of the mesh is great for a realistic visual display. However
when a mesh is used for a collision element, collision algorithms must process
all the polygons that are close to the target, which can be slow. For these
reasons it is often desirable to use something simpler than a mesh for the
collision model.  It's a tradeoff between accuracy of the shape and processing
time.

The ``<visual>`` tag is used in visualization programs like
``drake-visualizer`` (in the
`Director external <https://github.com/RobotLocomotion/director>`_). Drake does
not process the visual tag, unless you have something specific in your code that
will process it, like
`RgbdCamera <http://drake.mit.edu/doxygen_cxx/rgbd__camera_8h.html>`_.
Regardless of what program is processing the visual data, the processing time
of visual elements is generally not an issue.

In addition to modelling shapes with meshes, you can also model shapes with
geometric primitives, such as cylinders, spheres, or boxes. Geometric primitives
are far less complex than meshes and so require far less processing time.

In the `iiwa14.urdf robotic arm example
<https://github.com/RobotLocomotion/drake/blob/83740997e1c893be5d2209563b755cfe84ee1c32/drake/examples/kuka_iiwa_arm/urdf/iiwa14.urdf>`_,
we use meshes for the visual geometry elements, and
cylinders for the collision elements, except for the last links where meshes
are used.  The idea is that for links of the arm, we don't need exact shapes
for collision, where we don't expect to need precision. Even if we were to
use an "elbow" to shove an object out of the way, we probably don't need
exact accuracy.

But there are cases where we need the precision of a mesh. For example, extra
modeling precision may be needed to simulate a robot arm's end effector
reaching into tight spaces or performing dexterous manipulation tasks. In these
cases, if it's taking too long to process the original mesh, then the mesh needs
to be simplified. The next section describes some options.

.. _models_simplifying_meshes:

Simplifying a Mesh
==================

Q: What is the easiest way to use something simpler for a collision model?

A: If you are starting with a mesh of an object (e.g., from a 3D scanner),
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

An existing complex mesh can be simplified using a tool like `Blender
<https://www.blender.org/>`_, a free and open source 3D creation suite.
