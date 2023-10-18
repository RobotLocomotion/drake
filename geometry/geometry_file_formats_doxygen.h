/** @file
 Doxygen-only documentation for @ref geometry_file_formats.  */

namespace drake {
namespace geometry {

/** @addtogroup geometry_file_formats

 Drake offers a number of primitive types for representing objects' extents in
 space. They include:

   - Box
   - Capsule
   - Cylinder
   - Ellipsoid
   - HalfSpace
   - Sphere

 The primitives are good for coarse representations of real world shapes and,
 in some cases, may be preferred over more accurate representations for
 performance reasons (e.g., for @ref proximity_queries "proximity queries").
 However, there are cases where a higher fidelity representation is preferred.
 To this end, Drake also provides two more shapes:

   - Convex @ref file_footnote_1 "¹"
   - Mesh @ref file_footnote_1 "¹"

 Both of those Shape types reference on-disk geometry files. When a role is
 assigned to a geometry with the Convex or Mesh shape, the file will be parsed
 and some portion of the data in the file will be used. To make good
 representation choices, it is important to understand what kind of files
 are supported, in what roles, and to what degree. This page aims to make the
 answers to those questions clear and provide best practices. As geometry file
 format support is in flux the contents of this page will change frequently.

 @section supported_file_types Supported file types

 Drake is working towards a robust and consistent treatment of supported mesh
 types. During the transitional period, support will not be consistent. For each
 supported mesh type, we'll enumerate the nature of Drake's support for it,
 generally and with respect to each @ref geometry_roles "geometry role".

 You should assume that if a file format is not listed, it is not supported.

 Drake doesn't care about the capitalization of the geometry file extensions.

 Regarding the illustration role. While Drake has historically supported
 `drake_visualizer`, it no longer actively supports it. This documentation will
 focus on how the geometry is supported in
 <a href="https://github.com/meshcat-dev/meshcat">meshcat</a>.

 @subsection obj_support Wavefront (obj) file

 <a href=https://en.wikipedia.org/wiki/Wavefront_.obj_file>Wavefront obj</a> is
 a decades-old geometry format. The file is encoded in ASCII text with a simple
 syntax; simple objects can be created in a text editor. Most modern modeling
 packages can import and export .obj files. Its age has made it a universal
 format.

 The .obj specification includes an additional material library file (.mtl). The
 .mtl file defines materials that are applied to zero or more faces in the .obj
 file. The standard practice is for an .obj file to reference one or more .mtl
 files.

 Both Mesh and Convex shapes can accept an .obj file for any role (with
 limitations).

 - Illustration role
    - Both Mesh and Convex can be used with this role.
    - If the .obj file references an .mtl file (and that file is available) it
      will be used to define the materials.
    - Lack of surface normals may lead to unexpected rendering artifacts.
    - Lack of texture coordinates combined with a material with texture maps can
      likewise lead to unexpected rendering artifacts.

 - Perception role
    - The following notes apply to all Drake RenderEngine implementations.
    - The mesh must have surface normals (Drake throws if they are missing).
    - Texture coordinates are not required, but texture maps are only applied
      if an entire mesh "piece" has texture coordinates.
    - The material associated with the mesh is that defined by the
      @ref geometry_materials "material heuristic".
    - In contrast to the Proximity role, the mesh *can* have multiple objects
      and multiple materials. The obj's mesh data is partitioned into disjoint
      parts based on the applied material. The material heuristic is applied to
      each part individually.

 - Proximity role
    - The material file is ignored.
    - Drake currently requires the faces in the mesh to comprise a single
      object.
    - A Convex shape can be used to create a point-contact collision geometry or
      a hydroelastic collision geometry (either rigid or compliant).
        - Note: using a Convex shape implies that the .obj contains a single
          convex shape. This is currently not verified.
    - A Mesh shape can only be used to create a rigid hydroelastic collision
      geometry.

 - Best practices
    - Generally, make the .obj as complete as possible. If it's being used for
      either illustration or perception, include normals, texture coordinates,
      and materials as appropriate. Even if Drake doesn't complain about them
      missing, there are undefined behaviors across the set of visualizing
      technologies when they are missing.
    - Using .obj files usually produce objects with *simpler* materials. For
      materials with a greater fidelity, you might find it simpler to use a
      @ref gltf_support ".gltf" file instead.

 @subsection gltf_support glTFᵀᴹ (.gltf)

 A <a href="https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html">glTF</a>
 file is an "open interoperable 3D asset 'transmission' format". Like an .obj
 file it can include the definition of meshes and materials. glTF is designed
 to include data for an entire scene, including such diverse elements as nodes,
 animations, transform hierarchies, cameras, animations, morph targets, etc.
 Drake does not necessarily use all of that data.

 It's worth noting that glTF is designed, from the start, to support
 <a href="https://en.wikipedia.org/wiki/Physically_based_rendering">Physically
 Based Rendering (PBR)</a>. While the Wavefront .obj specification has informal
 extensions to achieve the same (replacing Phong with PBR), glTF uses it as its
 native shading model. This can produce images with higher fidelity, in both
 rasterization and path-tracing rendering algorithms.

 Generally, Drake provides broad support of glTF features. But there are some
 notable exceptions:

  - .glb files and .gltf files with external references (to image or .bin files)
    are not supported. Currently, Drake only supports .glTF files with all
    binary assets embedded (as buffers of base64-encoded data blobs).
  - Support for glTF extensions depends on the ultimate consumer of the glTF
    file (see below).
  - Animation and skinning. If such data is present, it is ignored.

 As with .obj files, both Mesh and Convex shapes can accept a .gltf file.

 - Illustration roles
     - .gltf features are well supported in meshcat-based visualizers (using
       either MeshcatVisualizer or DrakeVisualizer in conjunction with meldis).
     - There is generally broad support for glTF extensions.
     - To get optimal visual appearance, Meshcat should be configured to use an
       environment map (see Meshcat::SetEnvironmentMap()).

 - Perception roles
     - RenderEngineGl does not support .gltf files. A Mesh or Convex which
       references such a file will be silently ignored.
     - RenderEngineVtk does support .gltf files.
         - As VTK does not support extensions, RenderEngineVtk does not either.
         - RenderEngineVtk does not yet support specifying an environment map.
           This will lead to PBR materials being under illuminated.
     - RenderEngineGltfClient does support .gltf files (as the name suggests).
         - Geometries defined by a .gltf file are based almost verbatim to the
           render server (animation and morphing data is excluded).
         - The extensions are passed along, but whether or not they are
           supported depends on the server's implementation.

 - Proximity role
     - .gltf files are not currently supported for proximity roles at all.
       Assigning the proximity role to a Mesh with anything other than a .obj
       file will throw an exception.

 <!-- TODO(SeanCurtis-TRI): Flesh this out. Technically, Meshcat will consume
  the .dae file. But it will probably cause chaos if passed anywhere else.
 @subsection dae_support Collada file (.dae)

 Illustration role: it can be visualized via Meldis/Meshcat.
 -->

 <!-- TODO(20176) Fill out the details on best practices here.  -->
 @subsection vtk_support VTK file (.vtk)

 Drake uses tetrahedral meshes a couple of different ways: as compliant
 hydroelastic geometries (see @ref hug_title "here" for more details) and
 deformable bodies (see, e.g., multibody::DeformableModel). For some of the
 Shape types, Drake will provide a tetrahedralization of the shape at run time.
 However, Drake cannot yet do so for arbitrary surface meshes. Drake allows you
 to create your own tetrahedral mesh offline and request that Drake use it. We
 use the <a href="https://examples.vtk.org/site/VTKFileFormats/">.vtk</a> file
 format.

 Depending on the application (hydroelastics or deformable bodies), not every
 tetrahedralization is created equal. There are best practices for each domain
 which must be followed to get best results. More details on these best
 practices are coming.

 In addition to specifying the tetrahedral mesh for deformable or compliant
 hydroelastic geometries, the .vtk file can be used more generally:

   - When specified as Convex without hydroelastic properties, it produces the
     same representation for point contact as if you'd provided an .obj of
     just the surface polygons.
   - When specified as Mesh without hydroelastic properties, it is treated as
     a non-convex surface mesh; its convex hull is used for point contact.

  N.B. The .vtk file must still define a tetrahedral mesh and *not* a surface
  mesh.

 @section geometry_materials Drake materials and the specification heuristic

 Drake has a very rudimentary *internal* definition of visual materials. Don't
 mistake a "visual" material from a "dynamics" material. The latter consists of
 parameters such as elasticity, friction, etc. The former models how *light*
 interacts with the object including parameters such as diffuse color,
 specularity, etc. Drakes visual material model is based on the
 <a href="https://en.wikipedia.org/wiki/Phong_shading">Phong shading
 model</a>, but only makes use of a subset of that model's parameters.
 Currently, only the diffuse color property is parameterized. It can be either a
 single Rgba value, or a texture map.

 Even if a mesh file contains material definitions of its own, for some
 geometry operations, Drake will apply its own heuristic to define a Drake
 visual material. This may result in a visualization that appears different from
 the model itself. Drake is actively working on closing this gap.

 The heuristic defines a Phong material with the diffuse property defined with
 the following prioritized algorithm. The algorithm evaluates the following
 steps in sequence, but stops at the first step that provides a viable material
 definition.

  1. If the mesh file specifies materials, then the diffuse color or texture
     specified in that material will be applied.
     - If diffuse properties are *also* defined in the mesh's
       GeometryProperties, a warning will be written to the console.
     - If the material specifies a texture, but the mesh doesn't have texture
       coordinates, the texture is dismissed and only the material's diffuse
       color is used.
     - If the material specifies a texture, but the texture isn't accessible,
       the texture is dismissed and only the material's diffuse color is used.
     - <b>If the mesh file contains *multiple* materials, then all materials
       are discarded as if they weren't present and the next step of the
       algorithm is evaluated.</b>
  2. If the geometry in SceneGraph has had diffuse color or diffuse texture
     defined in its GeometryProperties, then that diffuse specification is
     used. The final diffuse color of the object is always the channel-wise
     product of the texture and the diffuse color. If the diffuse color is white
     the final diffuse color is defined solely by the texture. Otherwise, the
     final diffuse color can be a darkened texture (via a grey diffuse color)
     or tinted (via a diffuse color with a distinct hue). If the geometry
     properties define a texture but not a color, then the material is assigned
     a white diffuse color.
     - If a diffuse texture map is defined, but it isn't accessible, then only
       the white diffuse color is used.
  3. For a mesh file named my_mesh.file, if the file my_mesh.png is located in
     the same directory (and readable by Drake), it will be applied as an
     unmodulated diffuse texture.
  4. Default diffuse color defined by geometry consumer (visualizer or render
     engine) will be applied.

<!-- Foot notes for this file -->

<hr>

@anchor file_footnote_1
 ¹ The distinction between Mesh and Convex is only relevant for proximity
   queries; a Convex shape is treated differently from a Mesh shape. For
   illustration and perception roles, there is no practical distinction.

*/

}  // namespace geometry
}  // namespace drake
