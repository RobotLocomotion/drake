#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/render_gl/internal_opengl_geometry.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

/*
 This is tricky.

 On the one hand, each instance needs a unique copy of OpenGlGeometries because
 it has to recreate the VAO for each geometry and it *cannot* guarantee that
 the identifiers will match from context to context.

 However, I'd like the same benefit as the TextureLibrary where we don't
 duplicate meshes in GPU memory if two RenderEngineGl clones independently load
 the same mesh (shape or obj).

 The first requires independence. The second requires coupling.

 What if I guarantee that every copy uses the same VAO values?
  - The original RenderEngineGl creates N vertex array objects.
  - If we *assume* that the identifiers are created in sequence, then if we
    create a set of identifiers from 1 to the highest, I should be able to use
    every previously existing identifier has the same meaning in every clone.
  - The primary challenge is that if one instance adds a new geometry it only
    exists in *that* engine's context.
    - The other engines will see the geometry but be unable to use it because
      it doesn't have a corresponding context. :-P

 For now, we'll simply allow for redundancy with the understanding that most of
 the work happens during initialization so the odds of redundancy is small.
 Furthermore, if I pre-initialize the shape primitives, then I know the only
 redundancy will come from added meshes after cloning. */

/* Stores the OpenGlGeometry associated with a context.

 The *data* for each geometry, stored in the OpenGl context on the GPU, is
 shared between RenderEngineGl clones. However, the vertex array object (VAO)
 is *not* shared between clones. So, when a GeometryLibrary is cloned, the
 VAOs have to be reconstructed, using the shared geometry data (vertex buffers
 and index buffers). To that end, we define a custom copy constructor that
 handles updating the VAOs as part of the copy constructor.
 
 Every function in this class must only be invoked with an OpenGl context bound.
 It should be the context in which the defined geometries exist (or to which
 a geometry should be added if not already added).  */
class GeometryLibrary {
 public:
  GeometryLibrary() = default;

  GeometryLibrary(const GeometryLibrary& other);
  GeometryLibrary& operator=(const GeometryLibrary&) = delete;
  GeometryLibrary(GeometryLibrary&&) = delete;
  GeometryLibrary& operator=(GeometryLibrary&&) = delete;

  /* Returns the OpenGl geometry for a unit box (w = h = d = 1). */
  OpenGlGeometry& GetBox();

  /* Returns the OpenGl geometry for the capsule given.
   Note: one capsule cannot easily be converted to another capsule through
   simple affine transformations (scale, transform, rotate). So, N capsule
   requests create N unique geometries in the GPU's memory -- even if they are
   all identical. */
  OpenGlGeometry GetCapsule(const Capsule& capsule);

  /* Returns the OpenGl geometry for a unit cylinder (height and radius = 1). */
  OpenGlGeometry& GetCylinder();

  /* Returns the OpenGl geometry that approximates a half space -- a really big
   box with its +z facing face on the Gz = 0 plane. */
  OpenGlGeometry& GetHalfSpace();

  /* Returns the OpenGl geometry for the mesh indicates by the given path.
   @param default_diffuse   The color to apply to the mesh if it doesn't have
                            its own material definition. */
  OpenGlGeometry GetMesh(const std::filesystem::path mesh_path,
                         const Rgba& default_diffuse);

  /* Returns the OpenGl geometry for a unit sphere (radius = 1). */
  OpenGlGeometry& GetSphere();

 private:
  /* Creates an OpenGlGeometry from the mesh defined by the given mesh_data. */
  OpenGlGeometry CreateGlGeometry(
      const geometry::internal::RenderMesh& mesh_data) const;

  /* Given a geometry that has its buffers (and vertex counts assigned), ties
   all of the buffer data into the vertex array attributes. */
  void CreateVertexArray(OpenGlGeometry* geometry) const;

  /* One OpenGlGeometry per primitive type. They represent a canonical,
   "unit" version of the primitive type. Each instance scales and poses the
   corresponding primitive to create arbitrarily sized geometries.*/
  OpenGlGeometry box_;
  OpenGlGeometry cylinder_;
  OpenGlGeometry half_space_;
  OpenGlGeometry sphere_;

  // TODO(SeanCurtis-TRI): Figure out how to re-use capsules - if two capsules
  // have the same dimensions (or are related by a *uniform* scale*), we can
  // re-use the same geometry.
  /* Each capsule is unique; they cannot generally be related by a linear
   transform (i.e., translation, rotation, and non-uniform scale). */
  std::vector<OpenGlGeometry> capsules_;

  /* Mapping from obj filename to the mesh loaded into an OpenGlGeometry. */
  std::unordered_map<std::string, OpenGlGeometry> meshes_;
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
