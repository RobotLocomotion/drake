#include "drake/multibody/fem/dev/mesh_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeVertex;
using geometry::VolumeVertexIndex;
template <typename T>
VolumeMesh<T> CreateRectangularBlockTetMesh(int nx, int ny, int nz, T h) {
  /* Builds the elements. */
  std::vector<VolumeElement> elements;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) {
        /* For each block, the 8 corners are numerated as:
                             4*-----*7
                             /|    /|
                            / |   / |
                          5*-----*6 |
                           | 0*--|--*3
                           | /   | /
                           |/    |/
               j ^        1*-----*2
                 |
                 |
                 *-----> i
               /
              /
             k                                           */
        const VolumeVertexIndex p0((i * (ny + 1) + j) * (nz + 1) + k);
        const VolumeVertexIndex p1(p0 + 1);
        const VolumeVertexIndex p3(((i + 1) * (ny + 1) + j) * (nz + 1) + k);
        const VolumeVertexIndex p2(p3 + 1);
        const VolumeVertexIndex p7(((i + 1) * (ny + 1) + (j + 1)) * (nz + 1) +
                                   k);
        const VolumeVertexIndex p6(p7 + 1);
        const VolumeVertexIndex p4((i * (nx + 1) + (j + 1)) * (nz + 1) + k);
        const VolumeVertexIndex p5(p4 + 1);
        /* Ensure that neighboring tetrahedra are sharing faces, and we follow
         the convention that the first three vertices define a triangle with its
         right-handed normal pointing inwards. The fourth vertex is then on the
         positive side of this first triangle. */
        if ((i + j + k) % 2 == 1) {
          elements.emplace_back(p2, p1, p6, p3);
          elements.emplace_back(p6, p3, p4, p7);
          elements.emplace_back(p4, p1, p6, p5);
          elements.emplace_back(p3, p1, p4, p0);
          elements.emplace_back(p6, p1, p4, p3);
        } else {
          elements.emplace_back(p0, p2, p5, p1);
          elements.emplace_back(p7, p2, p0, p3);
          elements.emplace_back(p5, p2, p7, p6);
          elements.emplace_back(p7, p0, p5, p4);
          elements.emplace_back(p0, p2, p7, p5);
        }
      }
    }
  }
  /* Builds the vertices. */
  std::vector<VolumeVertex<T>> vertices;
  Vector3<T> position;
  for (int x = 0; x <= nx; ++x) {
    position(0) = h * x;
    for (int y = 0; y <= ny; ++y) {
      position(1) = h * y;
      for (int z = 0; z <= nz; ++z) {
        position(2) = h * z;
        vertices.emplace_back(position);
      }
    }
  }
  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}
template VolumeMesh<double> CreateRectangularBlockTetMesh(int, int, int,
                                                          double);
template VolumeMesh<AutoDiffXd> CreateRectangularBlockTetMesh(int, int, int,
                                                              AutoDiffXd);
}  // namespace fem
}  // namespace multibody
}  // namespace drake
