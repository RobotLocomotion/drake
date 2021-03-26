#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"

#include <utility>
#include <vector>

#include "drake/geometry/proximity/make_box_mesh.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
using geometry::Box;
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeVertex;
using geometry::VolumeVertexIndex;
/* Generates connectivity for the tetrahedral elements of the mesh by splitting
 each cube into five tetrahedra.
 @param[in] num_vertices
     Number of vertices in each of x-, y-, and z-directions.
 @return
     A sequence of tetrahedral elements that share unique vertices. */
std::vector<VolumeElement> GenerateDiamondCubicElements(
    const Vector3<int>& num_vertices) {
  std::vector<VolumeElement> elements;
  const Vector3<int> num_cells = num_vertices - Vector3<int>(1, 1, 1);
  /* The number of vertices in each direction. */
  for (int i = 0; i < num_cells(0); ++i) {
    for (int j = 0; j < num_cells(1); ++j) {
      for (int k = 0; k < num_cells(2); ++k) {
        /* The following picture shows where vertex vₛ (for `v[s]` below)
         locates in the rectangular cell.  The I-, J-, K-axes show the
         direction of increasing i, j, k indices.

                       v₁     v₃
                       ●------●
                      /|     /|
                     / |  v₇/ |
                 v₅ ●------●  |
                    |  |   |  |
                    |  ●---|--● v₂
                    | /v₀  | /
                    |/     |/
            +K   v₄ ●------● v₆
             |
             |
             o------+J
            /
           /
         +I                                                        */
        VolumeVertexIndex v[8];
        int s = 0;
        for (int l = 0; l < 2; ++l) {
          for (int m = 0; m < 2; ++m) {
            for (int n = 0; n < 2; ++n) {
              v[s++] =
                  VolumeVertexIndex(geometry::internal::CalcSequentialIndex(
                      i + l, j + m, k + n, num_vertices));
            }
          }
        }

        /* The following table subdivides the rectangular cell into five
         tetrahedra. Refer to the picture above to determine which four vertices
         form a tetrahedron. Refer to splitting No. 13 in:
         http://www.baumanneduard.ch/Splitting%20a%20cube%20in%20tetrahedras2.htm
         The splitting alternates depending on the positions of the cube to
         ensure that the mesh is conforming. */
        if ((i + j + k) % 2 == 1) {
          elements.emplace_back(v[6], v[4], v[7], v[2]);
          elements.emplace_back(v[7], v[2], v[1], v[3]);
          elements.emplace_back(v[1], v[4], v[7], v[5]);
          elements.emplace_back(v[2], v[4], v[1], v[0]);
          elements.emplace_back(v[7], v[4], v[1], v[2]);
        } else {
          elements.emplace_back(v[0], v[6], v[5], v[4]);
          elements.emplace_back(v[3], v[6], v[0], v[2]);
          elements.emplace_back(v[5], v[6], v[3], v[7]);
          elements.emplace_back(v[3], v[0], v[5], v[1]);
          elements.emplace_back(v[0], v[6], v[3], v[5]);
        }
      }
    }
  }
  return elements;
}

template <typename T>
VolumeMesh<T> MakeDiamondCubicBoxVolumeMesh(
    const Box& box, double resolution_hint,
    const math::RigidTransform<T>& X_WB) {
  DRAKE_DEMAND(resolution_hint > 0.);
  /* Number of vertices in x-, y-, and z- directions.  In each direction,
   there is one more vertices than cells. */
  const Vector3<int> num_vertices{
      1 + static_cast<int>(ceil(box.width() / resolution_hint)),
      1 + static_cast<int>(ceil(box.depth() / resolution_hint)),
      1 + static_cast<int>(ceil(box.height() / resolution_hint))};

  // Initially generate vertices in box's frame B.
  std::vector<VolumeVertex<T>> vertices =
      geometry::internal::GenerateVertices<T>(box, num_vertices);
  for (VolumeVertex<T>& vertex : vertices) {
    // Transform to World frame.
    vertex = VolumeVertex<T>(X_WB * vertex.r_MV());
  }

  std::vector<VolumeElement> elements =
      GenerateDiamondCubicElements(num_vertices);
  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

template VolumeMesh<double> MakeDiamondCubicBoxVolumeMesh(
    const geometry::Box&, double, const math::RigidTransform<double>&);
template VolumeMesh<AutoDiffXd> MakeDiamondCubicBoxVolumeMesh(
    const geometry::Box&, double, const math::RigidTransform<AutoDiffXd>&);
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
