/** @file
 Doxygen-only documentation for @ref geometry_file_formats.  */

namespace drake {
namespace geometry {

/** @addtogroup geometry_file_formats
 @{
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

 @section mesh_vs_convex Mesh vs Convex

 Both of those Shape types reference on-disk geometry files. When a role is
 assigned to a geometry with the Convex or Mesh shape, the file will be parsed
 and some portion of the data in the file will be used.

 The Convex shape is a declaration that the on-disk geometry file should only
 be used to compute a convex polytope (the convex hull of the vertices defined
 in the geometry file). The Convex shape is fully supported across all roles
 (illustration, perception, and proximity). For visual roles, whatever
 materials, normals, textures, etc. which may be specified in the referenced
 geometry file will not affect the visualized convex hull. It will always be
 a faceted, single-color polytope. The color is defined by assigning the
 ("phong", "diffuse") geometry property to the geometry. When it comes to file
 formats' compatibility with the Convex shape, the only question is if Drake
 has implemented the convex hull computation for the given file format. In the
 discussion below, each file format indicates whether Drake can compute its
 convex hull and, by implication, whether the file format can be referenced by
 Convex.

 The Mesh shape is a declaration that the on-disk geometry file should be used
 to the fullest extent that the role and geometry consumer can. For visual
 roles, that includes materials, textures, normals, etc. The degree to which
 the possible mesh data is used is outlined below.

 Things are slightly more complex with the proximity role. Performing contact
 and signed distance queries on general non-convex meshes is complicated. As
 such, Drake's implementation of Mesh support is incomplete. Where full support
 does not exist, the Mesh object is represented by its convex hull. Generally,
 full support is limited to a few cases:

  - @ref QueryObject::ComputeContactSurfaces "Hydroelastic contact" where the
    mesh has been declared "rigid".
  - @ref QueryObject::ComputeDeformableContact "Deformable contact" (the mesh
    serves as a rigid object which deformable objects can collide with).
  - @ref QueryObject::ComputeContactSurfaces "Hydroelastic contact" where the
    mesh has been declared "compliant", if and only if the referenced mesh file
    is a tetrahedral .vtk file. For all other mesh file types, see below.

 In these remaining cases, the Mesh's convex hull is used:

  - @ref QueryObject::ComputePointPairPenetration "Point contact"
  - @ref QueryObject::ComputeContactSurfaces "Hydroelastic contact" where the
    mesh has been declared "compliant". Unless the Mesh references a tetrahedral
    .vtk file (see note above).
  - @ref QueryObject::HasCollisions "Determining if collisions exist" at all.
  - @ref QueryObject::FindCollisionCandidates "Finding collision candidates"
  - Signed distance queries
    @ref QueryObject::ComputeSignedDistancePairwiseClosestPoints()
    "between all geometry pairs",
    @ref QueryObject::ComputeSignedDistancePairClosestPoints()
    "between an explicit geometry pair", or
    @ref QueryObject::ComputeSignedDistanceToPoint
    "between a geometry and a point".

 It is our *intent* that with time, more queries will move from the lower list
 to the fully-supported list.

 @section supported_file_types Supported file types

 To make good representation choices, it is important to understand what kind of
 files are supported, in what roles, and to what degree. This page aims to make
 the answers to those questions clear and provide best practices. As geometry
 file format support is in flux the contents of this page will change
 frequently.

 Drake is working towards a robust and consistent treatment of supported mesh
 types. During the transitional period, support will not be consistent. For each
 supported mesh type, we'll enumerate the nature of Drake's support for it,
 generally and with respect to each @ref geometry_roles "geometry role".

 You should assume that if a file format is not listed, it is not supported.

 Drake doesn't care about the capitalization of the geometry file extensions.

 This documentation will focus on how the geometry is supported in
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

 Drake can compute the convex hull for a .obj file, so it is fully compatible
 with Convex.

 A Mesh file which references an .obj file can be used with any role (with
 limitations).

 - Illustration role
    - Most, if not all, .obj features are supported.
    - If the .obj file references an .mtl file (and that file is available) it
      will be used to define the materials.
    - Lack of surface normals may lead to unexpected rendering artifacts.
    - Lack of texture coordinates combined with a material with texture maps can
      likewise lead to unexpected rendering artifacts.

 - Perception role
    - Most, if not all, .obj features are supported.
    - The following notes apply to all Drake RenderEngine implementations.
    - The mesh must have surface normals (Drake throws if they are missing).
    - Texture coordinates are not required, but texture maps are only applied
      if an entire mesh "piece" (collection of triangles with a single material)
      has texture coordinates.
    - The material associated with the mesh is that defined by the
      @ref geometry_materials "material heuristic".
    - In contrast to the Proximity role, the mesh *can* have multiple objects
      and multiple materials. The obj's mesh data is partitioned into disjoint
      parts based on the applied material. The material heuristic is applied to
      each part individually.

 - Proximity role
    - The material file is ignored.
    - As mentioned above, the Mesh's .obj will file will generally be replaced
      by its convex hull except for the two cases list above (rigid hydroelastic
      and deformable contact).
      - In these cases, the mesh need not be a closed manifold -- it could model
        an infinitely thin sheet.

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

 Drake can compute the convex hull for a .gltf file, so it is fully compatible
 with Convex.

 Generally, Drake provides broad support of glTF features. But there are some
 notable exceptions:

  - .glb "container" files are not supported.
  - Support for glTF
    <a href="https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#specifying-extensions">extensions</a>
    depends on the ultimate consumer of the glTF file (see below).
  - Animation and skinning. If such data is present, it is ignored.

 A Mesh file can reference a .gltf file but only for visual roles.

 - Illustration roles
    - .gltf features are well supported in meshcat-based visualizers (using
      either MeshcatVisualizer or DrakeVisualizer in conjunction with meldis).
    - There is generally
      <a href="https://threejs.org/docs/?q=loader#examples/en/loaders/GLTFLoader">broad support</a>
      for glTF extensions.
    - To get optimal visual appearance, Meshcat should be configured to use an
      environment map (see Meshcat::SetEnvironmentMap()).

 - Perception roles
    - RenderEngineGl has partial support for .gltf files. It doesn't support the
      full set of valid glTF files. We're not going to attempt to fully
      characterize what is/isn't supported and, instead, emphasize what will
      make the glTF file work best.
        - Limitations
          - RenderEngineGl uses simple Phong-based shaders. The PBR
            materials defined in the glTF will be automatically converted into
            an approximation. This approximation may change with time as the
            shaders mature. The appearance of PBR materials in RenderEngineGl is
            not part of Drake's API stability.
          - Drake requires the glTF to define vertex normals.
        - Supported features
          - Both embedded and external images.
          - Your glTF should indicate a
            <a href="https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#_gltf_scene">
            default scene</a>. It is *not* required and if absent,
            RenderEngineGl will use an arbitrary protocol for deciding which
            nodes in the glTF file will be rendered. This protocol is not part
            of the stable API. Making sure the `glTF.scene` value is well
            defined will provide long-term guarantees about what renders.
          - A single set of texture coordinates (the vertex attribute called
            "TEXCOORD_0").
          - glTF does not require specification of a material. If a mesh
            primitive doesn't indicate a material, the @ref geometry_materials
            "heuristic outlined below" will be applied.
    - RenderEngineVtk does support .gltf files.
        - The glTF extensions supported are whatever VTK's glTF loader
          implements, which at the time of this writing are KHR_lights_punctual
          and KHR_materials_unlit.
        - To get optimal visual appearance, RenderEngineVtk should be configured
          to use an environment map (see RenderEngineVtkParams::environment_map).
    - RenderEngineGltfClient does support .gltf files (as the name suggests).
        - Geometries defined by a .gltf file are passed almost verbatim to the
          render server (animation and morphing data is excluded).
        - The extensions are passed along, but whether or not they are
          supported depends on the server's implementation.

 - Proximity role
    - .gltf files *can* be used with the Convex shape type (as indicated
      above).
    - .gltf files *cannot* be used with the Mesh shape type. Assigning the
      proximity role to a Mesh which references a .gtlf file will throw an
      exception.

 As a rule of thumb, for Drake's purposes .gltf files with external assets
 (i.e., separate .bin and .png files) will load faster than .gltf files
 with assets embedded as base64-encoded data URIs.

 For best performance, we recommend that meshes destined for the Illustration
 role use the
 <a href="https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_texture_basisu/README.md">KHR_texture_basisu</a>
 extension, where textures are stored using the .ktx2 file format. Loading .ktx2
 textures in a web browser is substantially faster than .png textures. In the
 common case where the same mesh will also be used by the Perception role, be
 aware that many render engines do not support the KHR_texture_basisu extension
 so it should be listed in "extensionsUsed" but not "extensionsRequired" in the
 .gltf file for widest compatibility. (The mesh will have both .png and .ktx2
 textures available.) Refer to the
 <a href="https://github.com/RobotLocomotion/models">Drake models repository</a>
 for examples. To convert .png files to .ktx2 files, refer to the
 <a href="https://github.com/KhronosGroup/KTX-Software">KTX-Software</a>
 repository.

 <!-- TODO(SeanCurtis-TRI): Flesh this out. Technically, Meshcat will consume
  the .dae file. But it will probably cause chaos if passed anywhere else.
 @subsection dae_support Collada file (.dae)

 Illustration role: it can be visualized via Meldis/Meshcat.
 -->

 <!-- TODO(20176) Fill out the details on best practices here.  -->
 @subsection vtk_support VTK file (.vtk)

 Drake uses tetrahedral meshes a couple of different ways: as compliant
 hydroelastic geometries (see @ref hug_introduction "here" for more details) and
 deformable bodies (see, e.g., multibody::DeformableModel). For some of the
 Shape types (the mathematical primitives), Drake will provide a
 tetrahedralization of the shape at run time. However, Drake cannot yet do so
 for arbitrary surface meshes. Drake allows you to create your own tetrahedral
 mesh offline and request that Drake use it. We use the
 <a href="https://examples.vtk.org/site/VTKFileFormats/">.vtk</a> file
 format.

 Depending on the application (hydroelastics or deformable bodies), not every
 tetrahedralization is equally valid. There are best practices for each domain
 which must be followed to get best results. More details on these best
 practices are coming.

 Drake can compute the convex hull for a tetrahedral .vtk file, so it is fully
 compatible with Convex.

 In addition to specifying the tetrahedral mesh for deformable or compliant
 hydroelastic geometries, the .vtk file can be used more generally:

 - Illustration roles
    - A Mesh that references a tetrahedral .vtk file cannot have the
      illustration role assigned to it.
 - Perception roles
    - A Mesh that references a tetrahedral .vtk file cannot have the
      perception role assigned to it.
 - Proximity role
    - A Mesh that references a tetrahedral .vtk file can declared to have a
      *rigid* hydroelastic representation. In that case, the tetrahedral mesh's
      surface is used (as a rigid triangle mesh).
    - When specified as Mesh without hydroelastic properties, it is treated as
      a non-convex surface mesh; its convex hull is used as documented above.
  <!-- TODO(xuchenhan-tri) When deformable goes public, we should indicate that
   we can use this file type to declare a Mesh that is a *deformable* geo. -->

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
     specified in that material will be applied. Note: cases where an .obj
     references a material name, but the material is not defined in the .mtl
     file, or the .mtl file is missing, will be treated as if no material is
     specified and we proceed to step 2.
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

 @section geometry_image_types Supported image types

 Drake supports four main image file formats: PNG, JPG, TIFF, and HDR (there is
 an additional format, KTX2, see below).

 PNG, JPG, and TIFF: can be written (via systems::sensors::ImageIo and
 systems::sensors::ImageWriter). They can be read as part of model definitions
 (e.g. as part of glTF or Obj objects). They can be used to define environment
 maps in geometry::Meshcat::SetEnvironmentMap() or
 geometry::RenderEngineVtkParams.

 HDR: This file type can only be used to define environment maps in
 geometry::Meshcat::SetEnvironmentMap() or geometry::RenderEngineVtkParams.

 KTX2: This file type can only be used as part of a glTF file (see above). It is
 a highly compressed image format designed to be used efficiently by GPUs. It is
 currently only supported by Meshcat (via the illustration role) and will either
 be ignored or cause warnings to be logged if used elsewhere. (See
 @ref gltf_support "the discussion of glTF files" above.")

<!-- Foot notes for this file -->

<hr>

@anchor file_footnote_1
 ¹ The distinction between Mesh and Convex is only relevant for proximity
   queries; a Convex shape is treated differently from a Mesh shape. For
   illustration and perception roles, there is no practical distinction.
@}
*/

}  // namespace geometry
}  // namespace drake
