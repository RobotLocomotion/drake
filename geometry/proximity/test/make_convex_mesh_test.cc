#include "drake/geometry/proximity/make_convex_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Fixture to create a compliant mesh type from a simple cube mesh obj file
// and a more compliacated general convex mesh obj.
class MakeConvexMeshTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string cube_file =
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    std::string convex_file =
        FindResourceOrThrow("drake/geometry/test/convex.obj");

    cube_tri_mesh_ = std::make_unique<TriangleSurfaceMesh<double>>(
        ReadObjToTriangleSurfaceMesh(cube_file));
    convex_tri_mesh_ = std::make_unique<TriangleSurfaceMesh<double>>(
        ReadObjToTriangleSurfaceMesh(convex_file));

    cube_mesh_ = std::make_unique<VolumeMesh<double>>(
        MakeConvexVolumeMesh<double>(*cube_tri_mesh_));
    convex_mesh_ = std::make_unique<VolumeMesh<double>>(
        MakeConvexVolumeMesh<double>(*convex_tri_mesh_));
  }

 protected:
  std::unique_ptr<TriangleSurfaceMesh<double>> cube_tri_mesh_;
  std::unique_ptr<TriangleSurfaceMesh<double>> convex_tri_mesh_;
  std::unique_ptr<VolumeMesh<double>> cube_mesh_;
  std::unique_ptr<VolumeMesh<double>> convex_mesh_;
};

// Tests that two triangle surface meshes are "equal" in the sense that their
// exists a bijection between the vertices of the two meshes and a bijection
// between the faces of the meshes (with consistent face normal orientation).
void TestSurfaceMeshEquality(const TriangleSurfaceMesh<double>& mesh_A,
                             const TriangleSurfaceMesh<double>& mesh_B) {
  // Firstly the meshes must contain the same number of vertices and elements.
  EXPECT_EQ(mesh_A.num_vertices(), mesh_B.num_vertices());
  EXPECT_EQ(mesh_A.num_elements(), mesh_B.num_elements());

  {
    // Construct a mapping between the vertices of mesh_A and mesh_B ensure
    // the mapping is a bijection.

    // Vertex equivalence is just numerical equivalence of coordinates.
    auto find_vertex = [mesh_B](const Eigen::Vector3d& v) {
      for (int i = 0; i < mesh_B.num_vertices(); ++i) {
        if (v == mesh_B.vertex(i)) return i;
      }
      return -1;
    };

    std::vector<int> vertex_mapping(mesh_A.num_vertices(), -1);
    for (int i = 0; i < mesh_A.num_vertices(); ++i) {
      vertex_mapping[i] = find_vertex(mesh_A.vertex(i));
    }

    // All vertices of mesh_A should match to some vertex of mesh_B
    EXPECT_TRUE(std::find(vertex_mapping.begin(), vertex_mapping.end(), -1) ==
                vertex_mapping.end());

    // If the mapping is equivalent to its unique elements and doesn't contain
    // -1, then the mapping is a bijection.
    auto unique_it = std::unique(vertex_mapping.begin(), vertex_mapping.end());
    int unique_size = static_cast<int>(unique_it - vertex_mapping.begin());
    EXPECT_EQ(mesh_A.num_vertices(), unique_size);
  }

  {
    // Construct a mapping between the elements of mesh_A and mesh_B ensure
    // the mapping is a bijection.

    // Two elements are equal if they contain the same vertex indices and have
    // the same winding order (face normals are in the same direction).
    auto find_element = [mesh_A, mesh_B](int index) {
      SurfaceTriangle t = mesh_A.element(index);
      SortedTriplet t_sorted(t.vertex(0), t.vertex(1), t.vertex(2));
      for (int i = 0; i < mesh_B.num_elements(); ++i) {
        const SurfaceTriangle& u = mesh_B.element(i);
        SortedTriplet u_sorted(u.vertex(0), u.vertex(1), u.vertex(2));
        if (t_sorted == u_sorted &&
            mesh_A.face_normal(index).dot(mesh_B.face_normal(i)) > 0) {
          return i;
        }
      }
      return -1;
    };

    std::vector<int> element_mapping(mesh_A.num_elements(), -1);

    for (int i = 0; i < mesh_A.num_elements(); ++i) {
      element_mapping[i] = find_element(i);
    }

    // All faces of mesh_A should match to some face of mesh_B
    EXPECT_TRUE(std::find(element_mapping.begin(), element_mapping.end(), -1) ==
                element_mapping.end());

    // If the mapping is equivalent to its unique elements and doesn't contain
    // -1, then the mapping is a bijection.
    auto unique_it =
        std::unique(element_mapping.begin(), element_mapping.end());
    int unique_size = static_cast<int>(unique_it - element_mapping.begin());
    EXPECT_EQ(mesh_A.num_elements(), unique_size);
  }
}

// Tests that the vertices of the produced volume mesh are coherent with the
// current scheme for producing a volume mesh from a convex triangle surface
// mesh. Currently the scheme uses each vertex of the input mesh and creates
// one internal vertex that closes the volume. This tests that all vertices
// except the final vertex of the created volume mesh exist in the triangle
// mesh as well as testing that the single internal vertex is indeed internal.
void TestCoherentVertices(const TriangleSurfaceMesh<double>& tri_mesh,
                          const VolumeMesh<double>& vol_mesh) {
  const Eigen::Vector3d& internal_vertex = vol_mesh.vertices().back();

  auto find_vertex = [tri_mesh](const Eigen::Vector3d& v) {
    for (int i = 0; i < tri_mesh.num_vertices(); ++i) {
      if (v == tri_mesh.vertex(i)) {
        return i;
      }
    }
    return -1;
  };

  // All but the last vertex should exist in the triangle mesh.
  for (int i = 0; i < vol_mesh.num_vertices() - 1; ++i) {
    EXPECT_NE(find_vertex(vol_mesh.vertex(i)), -1);
  }

  EXPECT_EQ(find_vertex(internal_vertex), -1);

  // The last vertex should be on the interior of the triangle mesh.
  for (int i = 0; i < tri_mesh.num_elements(); ++i) {
    const Eigen::Vector3d p =
        tri_mesh.vertex(tri_mesh.element(i).vertex(0)) - internal_vertex;
    EXPECT_GE(tri_mesh.face_normal(i).dot(p), 0);
  }
}

TEST_F(MakeConvexMeshTest, CoherhentVertices) {
  TestCoherentVertices(*cube_tri_mesh_, *cube_mesh_);
  TestCoherentVertices(*convex_tri_mesh_, *convex_mesh_);
}

TEST_F(MakeConvexMeshTest, SurfaceMeshEqualInput) {
  const TriangleSurfaceMesh<double> converted_cube_tri_mesh =
      ConvertVolumeToSurfaceMesh(*cube_mesh_);
  TestSurfaceMeshEquality(*cube_tri_mesh_, converted_cube_tri_mesh);

  const TriangleSurfaceMesh<double> converted_convex_tri_mesh =
      ConvertVolumeToSurfaceMesh(*convex_mesh_);
  TestSurfaceMeshEquality(*convex_tri_mesh_, converted_convex_tri_mesh);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
