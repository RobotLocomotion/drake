#include "drake/geometry/proximity/make_convex_mesh.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Fixture to create a compliant mesh type from a simple cube mesh obj file
// and a more complicated non-convex mesh (only for using as Convex shape).
class MakeConvexMeshTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string cube_file =
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    std::string hole_cube_file =
        FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj");

    const Mesh cube_spec(cube_file);
    cube_tri_mesh_ = std::make_unique<TriangleSurfaceMesh<double>>(
        internal::MakeTriangleFromPolygonMesh(cube_spec.GetConvexHull()));
    const Mesh hole_cube_spec(hole_cube_file);
    hole_tri_mesh_ = std::make_unique<TriangleSurfaceMesh<double>>(
        internal::MakeTriangleFromPolygonMesh(hole_cube_spec.GetConvexHull()));

    cube_tet_mesh_ = std::make_unique<VolumeMesh<double>>(
        MakeConvexVolumeMesh<double>(Convex(cube_file)));
    hole_cube_tet_mesh_ = std::make_unique<VolumeMesh<double>>(
        MakeConvexVolumeMesh<double>(Convex(hole_cube_file)));
  }

 protected:
  std::unique_ptr<TriangleSurfaceMesh<double>> cube_tri_mesh_;
  std::unique_ptr<TriangleSurfaceMesh<double>> hole_tri_mesh_;
  std::unique_ptr<VolumeMesh<double>> cube_tet_mesh_;
  std::unique_ptr<VolumeMesh<double>> hole_cube_tet_mesh_;
};

// Tests that two triangle surface meshes are "equal" in the sense that their
// exists a bijection between the vertices of the two meshes and a bijection
// between the faces of the meshes (with consistent face normal orientation).
// TODO(DamrongGuoy): Refactor to make this available outside this unit test.
//
// @pre Vertices in each mesh are unique. Each mesh has no duplicated vertices.
//
// TODO(joemasterjohn): Consider using gtest's container-matchers to make this
// helper function more concise.
void TestSurfaceMeshEquality(const TriangleSurfaceMesh<double>& mesh_A,
                             const TriangleSurfaceMesh<double>& mesh_B) {
  // Firstly the meshes must contain the same number of vertices and elements.
  ASSERT_EQ(mesh_A.num_vertices(), mesh_B.num_vertices());
  ASSERT_EQ(mesh_A.num_elements(), mesh_B.num_elements());

  // Construct a mapping between the vertices of mesh_A and mesh_B ensure
  // the mapping is a bijection.

  // Vertex equivalence is just numerical equivalence of coordinates.
  auto find_vertex = [&mesh_B](const Eigen::Vector3d& vertex_coordinates) {
    for (int i = 0; i < mesh_B.num_vertices(); ++i) {
      if (vertex_coordinates == mesh_B.vertex(i)) return i;
    }
    return -1;
  };

  std::vector<int> vertex_mapping(mesh_A.num_vertices(), -1);
  for (int i = 0; i < mesh_A.num_vertices(); ++i) {
    vertex_mapping[i] = find_vertex(mesh_A.vertex(i));
  }

  // All vertices of mesh_A should match to some vertex of mesh_B
  ASSERT_TRUE(std::find(vertex_mapping.begin(), vertex_mapping.end(), -1) ==
              vertex_mapping.end());

  // If the mapping is equivalent to its unique elements and doesn't contain
  // -1, then the mapping is a bijection.
  std::vector<int> sorted_vertex_mapping(vertex_mapping);
  std::sort(sorted_vertex_mapping.begin(), sorted_vertex_mapping.end());
  auto unique_vertex_it =
      std::unique(sorted_vertex_mapping.begin(), sorted_vertex_mapping.end());
  int unique_vertex_size =
      static_cast<int>(unique_vertex_it - sorted_vertex_mapping.begin());
  ASSERT_EQ(mesh_A.num_vertices(), unique_vertex_size);

  // Construct a mapping between the elements of mesh_A and mesh_B ensure
  // the mapping is a bijection.

  // Two elements are equal if they contain the same vertex indices and have
  // the same winding order (face normals are in the same direction).
  auto find_element = [&mesh_A, &mesh_B, &vertex_mapping](int index) {
    SurfaceTriangle t = mesh_A.element(index);
    SortedTriplet t_sorted(vertex_mapping[t.vertex(0)],
                           vertex_mapping[t.vertex(1)],
                           vertex_mapping[t.vertex(2)]);
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
  ASSERT_TRUE(std::find(element_mapping.begin(), element_mapping.end(), -1) ==
              element_mapping.end());

  // If the mapping is equivalent to its unique elements and doesn't contain
  // -1, then the mapping is a bijection.
  std::vector<int> sorted_element_mapping(element_mapping);
  std::sort(element_mapping.begin(), element_mapping.end());
  auto unique_element_it =
      std::unique(sorted_element_mapping.begin(), sorted_element_mapping.end());
  int unique_element_size =
      static_cast<int>(unique_element_it - sorted_element_mapping.begin());
  ASSERT_EQ(mesh_A.num_elements(), unique_element_size);
}

// Tests that the vertices of the produced volume mesh are coherent with the
// current scheme for producing a volume mesh from a convex triangle surface
// mesh. Currently the scheme uses each vertex of the input mesh and creates
// one internal vertex that closes the volume. This tests that all vertices
// except the final vertex of the created volume mesh exist in the triangle
// mesh as well as testing that the single internal vertex is indeed internal.
//
// @pre The input `tri_mesh` has outward face normals.
void TestCoherentVertices(const TriangleSurfaceMesh<double>& tri_mesh,
                          const VolumeMesh<double>& vol_mesh) {
  const Eigen::Vector3d& internal_vertex = vol_mesh.vertices().back();

  auto find_vertex = [&tri_mesh](const Eigen::Vector3d& v) {
    for (int i = 0; i < tri_mesh.num_vertices(); ++i) {
      if (v == tri_mesh.vertex(i)) {
        return i;
      }
    }
    return -1;
  };

  // All but the last vertex should exist in the triangle mesh.
  for (int i = 0; i < vol_mesh.num_vertices() - 1; ++i) {
    ASSERT_NE(find_vertex(vol_mesh.vertex(i)), -1);
  }

  ASSERT_EQ(find_vertex(internal_vertex), -1);

  // The last vertex should be in the interior of the triangle mesh.
  for (int i = 0; i < tri_mesh.num_elements(); ++i) {
    const Eigen::Vector3d p =
        tri_mesh.vertex(tri_mesh.element(i).vertex(0)) - internal_vertex;
    ASSERT_GE(tri_mesh.face_normal(i).dot(p), 0);
  }
}

TEST_F(MakeConvexMeshTest, CoherentVertices) {
  TestCoherentVertices(*cube_tri_mesh_, *cube_tet_mesh_);
  TestCoherentVertices(*hole_tri_mesh_, *hole_cube_tet_mesh_);
}

TEST_F(MakeConvexMeshTest, SurfaceMeshEqualInput) {
  {
    SCOPED_TRACE("Cube mesh");
    const TriangleSurfaceMesh<double> converted_cube_tri_mesh =
        ConvertVolumeToSurfaceMesh(*cube_tet_mesh_);
    TestSurfaceMeshEquality(*cube_tri_mesh_, converted_cube_tri_mesh);
  }

  {
    SCOPED_TRACE("Hole cube mesh");
    const TriangleSurfaceMesh<double> converted_convex_tri_mesh =
        ConvertVolumeToSurfaceMesh(*hole_cube_tet_mesh_);
    TestSurfaceMeshEquality(*hole_tri_mesh_, converted_convex_tri_mesh);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
