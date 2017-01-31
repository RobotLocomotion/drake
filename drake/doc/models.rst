
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
Drake can accept the following mesh file formats:

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

.. _models_contents:

The content of a Model file
===========================

For a simple example, see `this cinder block example
<https://github.com/RobotLocomotion/drake/blob/9a67372fa00054aedf8a9f30684bdd5dc2ee9b0d/drake/examples/Atlas/sdf/cinder_block_2/model-1_4.sdf>`_.
As you can see, an SDF file is an XML file.

The way the model looks in an SDF file is given by the "visual" tag. In the
example, it looks like this::

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://cinder_block_2/meshes/cinder_block.dae</uri>
          </mesh>
        </geometry>
      </visual>

The visual is defined to be a mesh given by `this DAE file
<https://github.com/RobotLocomotion/drake/blob/9a67372fa00054aedf8a9f30684bdd5dc2ee9b0d/drake/examples/Atlas/sdf/cinder_block_2/meshes/cinder_block.dae>`_.
The DAE mesh file defines many
things concerning the look of the object, including a reference to a PNG file
which defines the texture. If you open the PNG file in an image viewing program
like `GIMP <https://www.gimp.org>`_, you will see it shows the pitted texture
and grey color of a cinder block.

While the visual properties define how the model looks, often we need to
describe the shape of the hard parts of the model (the rigid bodies), which
the SDF file defines with a "collision" tag. You may wonder why we don't use
the visual mesh for the collision element.  That is possible, and it would be
accurate, but a visual mesh is usually very detailed and complex, making it
computationally expensive to use as a collision model.

A much simpler way to describe a shape is with geometric primitives, such as
cylinders, spheres, or boxes. The complexity of a geometric primitive
is far less than with a mesh. If you look at the cinder block SDF file,
you'll see that the geometries of the collision elements are defined as
boxes, instead of using the mesh. In this cinder block, the outline of the
object is a rectangle (ignoring the holes in the cinder block), so we can
use the box shape.

See the `URDF <urdf/drakeURDF.html#://>`_ or `SDF <http://sdformat.org/>`_
references for more information.

The best way to learn how to create a model is to examine and change an
existing, working model.  `This tutorial <https://www.youtube
.com/watch?v=gugV8IMyHnY>`_ walks through editing a model in Drake.

.. _models_simplifying_meshes:

Simplifying a Mesh
==================

Meshes of complicated objects contain many vertices. The detail (density) of
the mesh makes the visual representation look realistic. However when a mesh
is used for a collision element, collision algorithms must process the entire
mesh, which can be slow. For these reasons it is often desirable for the
collision model to use a simpler structure than the original high density
mesh.

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

In the `iiwa14.urdf robotic arm example
<https://github.com/RobotLocomotion/drake/blob/83740997e1c893be5d2209563b755cfe84ee1c32/drake/examples/kuka_iiwa_arm/urdf/iiwa14.urdf>`_,
we use meshes for the visual geometry elements, and
cylinders for the collision elements, except for the last links where meshes
are used.  The idea is that for links of the arm, we don't need exact shapes
for collision, where we don't expect to need precision. Even if we were to
use an "elbow" to shove an object out of the way, we probably don't need
exact accuracy. It's possible we may desire more precision for the links at
the end of the arm, if the arm is reaching into tight or crowded spaces or
performing dexterous manipulation tasks.

An existing complex mesh can be simplified using a tool like `Blender
<https://www.blender.org/>`_, a free and open source 3D creation suite.

