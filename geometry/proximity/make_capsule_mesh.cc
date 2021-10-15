#include "drake/geometry/proximity/make_capsule_mesh.h"

#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
VolumeMesh<T> MakeCapsuleVolumeMesh(const Capsule& capsule,
                                    double resolution_hint) {
  // Z coordinates of the endpoints of the medial axis.
  const double medial_top_z = capsule.length() / 2.;
  const double medial_bottom_z = -medial_top_z;
  // Z coordinates of the poles of the caps.
  const double top_z = medial_top_z + capsule.radius();
  const double bottom_z = -top_z;

  // Let n be the number of vertices on each circular rim of the capsule.
  // The total number of vertices, V = 2n⌊n/2⌋ + 4
  // The total number of tetrahedra, F = 4n(⌊n/2⌋ - 1) + 5n
  // We need to be able to index both the vertices and tetrahedra with type safe
  // indices, thus the absolute maximum number of tetrahedra is INT_MAX. However
  // we choose a more reasonable maximum of 1 million tetrahedra.
  //
  // F = 4n(⌊n/2⌋ - 1) + 5n ≤ MAX
  // F ≤ 4n(n/2 - 1) + 5n ≤ MAX
  // 2n² + n - MAX ≤ 0
  // n ≤ ((√ 8 MAX + 1) - 1) / 4
  // n ≤ ⌊ 706.86... ⌋  for MAX = 1'000'000
  // n ≤ 706

  // The mesh will have a minimum of 3 vertices on its rim.
  const int num_vertices_per_circle = static_cast<int>(
      std::clamp(2. * M_PI * capsule.radius() / resolution_hint, 3.0, 706.0));

  // Each vertex on the rim corresponds to a line of longitude on the
  // hemispheres of the caps. num_circles_per_cap is how many lines of latitude
  // to subdivide each cap into.
  const int num_circles_per_cap = num_vertices_per_circle / 2;

  std::vector<Vector3<T>> mesh_vertices;
  // 2 caps * number of circles per cap * number of verts per circle
  // + the two medial vertices and the two cap pole vertices.
  mesh_vertices.reserve(2 * num_circles_per_cap * num_vertices_per_circle + 4);

  // Add the cap pole vertices and the medial vertices.
  int medial_top = static_cast<int>(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, medial_top_z);
  int medial_bottom = static_cast<int>(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, medial_bottom_z);
  int top = static_cast<int>(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, top_z);
  int bottom = static_cast<int>(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, bottom_z);

  // Storage for the vertex indices of each cap.
  std::vector<int> top_cap(num_circles_per_cap * num_vertices_per_circle);
  std::vector<int> bottom_cap(num_circles_per_cap * num_vertices_per_circle);

  // The vertices of the caps are spaced in an equal subdivision of the sphere
  // with spherical coordinates. Vertex (i, j) will have spherical coordinates:
  // (r, θ, φ) = (radius, π - i*theta_step, j*phi_step) where θ is the polar
  // angle and φ the azimuthal angle. One of the peculiarities of this
  // parameterization is the anisotropic behavior of contact patch resolution
  // at different points of the sphere. For instance, the pole vertices have
  // a degree of `num_vertices_per_circle` while all other vertices on the
  // sphere have a degree of 4. Thus a shallow contact with a plane at and
  // tangent to the pole will produce a much denser contact surface than that
  // produced by the same type of shallow contact at a point at the equator
  // of the sphere. It is unclear at this moment whether that artifact has an
  // effect on the simulation.
  const double theta_step = M_PI_2 / num_circles_per_cap;
  const double phi_step = 2 * M_PI / num_vertices_per_circle;

  // Subdivide the sphere and partition into top and bottom caps.
  for (int i = 0; i < num_circles_per_cap; ++i) {
    const double theta = M_PI_2 - i * theta_step;
    const double s = sin(theta);
    // Offset the z value of each cap.
    const double top_circle_z = capsule.radius() * cos(theta) + medial_top_z;
    const double bottom_circle_z = -top_circle_z;

    for (int j = 0; j < num_vertices_per_circle; ++j) {
      const double phi = j * phi_step;
      const double x = capsule.radius() * s * cos(phi);
      const double y = capsule.radius() * s * sin(phi);

      top_cap[num_vertices_per_circle * i + j] =
          static_cast<int>(mesh_vertices.size());
      mesh_vertices.emplace_back(x, y, top_circle_z);
      bottom_cap[num_vertices_per_circle * i + j] =
          static_cast<int>(mesh_vertices.size());
      mesh_vertices.emplace_back(x, y, bottom_circle_z);
    }
  }

  std::vector<VolumeElement> mesh_elements;

  // Each quadrilateral of each hemispherical cap is the base of a pyramid
  // whose top vertex is the center of the sphere of the cap (one of the medial
  // vertices.) The pyramids are split into tetrahedra in such a way that they
  // conform to each other. That is to say they share common faces where they
  // meet by the appropriate choice of diagonal. The quadrilateral faces of the
  // pyramid are: [(i,j), (i, j+1), (i+1, j+1), (i+1, j)] where those vertices
  // are given counterclockwise when viewed from outside of the capsule.
  //
  //      (i+1,j)  ______
  //             o        +
  //          ／   \         ＼
  //         /      \         \   (one slice of the top cap)
  // (i,j)  o---------o M₁     +

  // i indexes latitude and j indexes longitude.
  for (int i = 0; i < num_circles_per_cap - 1; ++i) {
    for (int j = 0; j < num_vertices_per_circle; ++j) {
      // Connect to the next longitude, wrapping around the circle.
      const int j1 = (j + 1) % num_vertices_per_circle;

      // The arguments to SplitPyramidToTetrahedra are given in CLOCKWISE order
      // for the base.
      Append(SplitPyramidToTetrahedra(
                 top_cap[(i + 1) * num_vertices_per_circle + j],
                 top_cap[(i + 1) * num_vertices_per_circle + j1],
                 top_cap[i * num_vertices_per_circle + j1],
                 top_cap[i * num_vertices_per_circle + j], medial_top),
             &mesh_elements);

      Append(
          SplitPyramidToTetrahedra(
              bottom_cap[i * num_vertices_per_circle + j],
              bottom_cap[i * num_vertices_per_circle + j1],
              bottom_cap[(i + 1) * num_vertices_per_circle + j1],
              bottom_cap[(i + 1) * num_vertices_per_circle + j], medial_bottom),
          &mesh_elements);
    }
  }

  // The vertices on the last circle of latitude on each cap all connect to the
  // pole vertices forming a circular cap divided into triangular wedges. Each
  // of those wedges connects to the corresponding medial vertex, forming a
  // single tetrahedra.
  //
  // The two equatorial circles of each cap connect corresponding vertices to
  // each other, forming the cylindrical middle portion of the capsule. Each
  // quadrilateral face of the cylinder connects to the two medial vertices,
  // forming a triangular prism. Each prism is then subdivided into 3 tetrahedra
  // such that adjacent prisms are conforming.

  // Offset into the index vector of the last circle of the caps.
  const int last_circle_offset =
      (num_circles_per_cap - 1) * num_vertices_per_circle;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    const int j1 = (j + 1) % num_vertices_per_circle;

    // Add tetrahedra of the top and bottom caps.
    // Vertices are oriented such that the signed volume of the tetrahedra
    // is positive, assuming the right hand rule.
    mesh_elements.emplace_back(top, top_cap[last_circle_offset + j1],
                               top_cap[last_circle_offset + j], medial_top);

    mesh_elements.emplace_back(bottom, bottom_cap[last_circle_offset + j],
                               bottom_cap[last_circle_offset + j1],
                               medial_bottom);

    // Add the cylindrical prisms.
    // Vertices are passed in the order: bottom triangular cap (oriented
    // inwards) and then top triangular cap (oriented outwards).
    Append(SplitTriangularPrismToTetrahedra(medial_bottom, bottom_cap[j],
                                            bottom_cap[j1], medial_top,
                                            top_cap[j], top_cap[j1]),
           &mesh_elements);
  }

  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &MakeCapsuleVolumeMesh<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
