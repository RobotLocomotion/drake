#include "drake/geometry/proximity/polygon_surface_mesh.h"

#include <numeric>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::vector;

/* Simple struct for defining an expected polygon. It doesn't do math and
 assumes that the various quantities are defined consistently. */
struct Polygon {
  std::string name;
  vector<Vector3d> vertices;
  double area;
  Vector3d normal;
  Vector3d centroid;
};

template <typename T>
class PolygonSurfaceMeshTest : public ::testing::Test {
 protected:
  /* Adds the given polygon to the raw vertex and face data for exercising the
   advanced constructor. */
  void AddPolygon(const Polygon& polygon, vector<int>* face_data,
                  vector<Vector3<T>>* vertices) {
    const int first_new_index = static_cast<int>(vertices->size());
    for (const auto& v : polygon.vertices) {
      vertices->emplace_back(v.cast<T>());
    }
    const int polygon_size = static_cast<int>(polygon.vertices.size());
    vector<int> v_indices(polygon_size);
    std::iota(v_indices.begin(), v_indices.end(), first_new_index);
    face_data->push_back(polygon_size);
    face_data->insert(face_data->end(), v_indices.begin(), v_indices.end());
  }

  /* Returns an isosceles triangle with a known area and normal, posed in
   Frame W. */
  Polygon MakeIsoscelesTriangle(const RigidTransformd& X_WM = {}) {
    vector<Vector3d> vertices{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    Vector3d centroid(0, 0, 0);
    for (auto& p_MV : vertices) {
      p_MV = X_WM * p_MV;
      centroid += p_MV;
    }
    centroid /= 3;
    Vector3d normal_W = X_WM.rotation() * Vector3d{0, 0, 1};
    return {.name = "Isosceles triangle",
            .vertices = vertices,
            .area = 0.5,
            .normal = normal_W,
            .centroid = centroid};
  }

  /* Returns a simple square polygon posed in frame W. */
  Polygon MakeSquare(const RigidTransformd& X_WM = {}) {
    const double h = 0.25;
    vector<Vector3d> vertices{{0, 0, 0}, {h, 0, 0}, {h, 0, h}, {0, 0, h}};
    for (auto& p_MV : vertices) {
      p_MV = X_WM * p_MV;
    }
    Vector3d normal_W = X_WM.rotation() * Vector3d{0, -1, 0};
    return {.name = "Square",
            .vertices = vertices,
            .area = h * h,
            .normal = normal_W,
            .centroid = X_WM * Vector3d(h / 2, 0, h / 2)};
  }

  /* Returns a regular pentagon posed in frame W. */
  Polygon MakePentagon(const RigidTransformd& X_WM = {}) {
    /* We'll define the pentagon by simply sampling a circle of radius R at
     2π/5 intervals. These five points are the polygon vertices. They'll lie on
     the yz-plane (so the normal is in the x-direction). The area is computed
     as the sum of the areas of five isosceles triangles.

               θ         θ/2 = π/5
              /|\          w = R⋅sin(θ/2)
           R / | \         h = R⋅cos(θ/2)
            /  |  \      A = R²⋅sin(θ/2)⋅cos(θ/2)
           /  h|   \
          /____|____\
             w                                       */
    vector<Vector3d> vertices;
    const double kR = 0.75;
    const double kTheta = M_PI * 2 / 5;
    for (int i = 0; i < 5; ++i) {
      const double alpha = i * kTheta;
      vertices.emplace_back(
          X_WM * Vector3d(0, kR * std::cos(alpha), kR * std::sin(alpha)));
    }
    return {.name = "Pentagon",
            .vertices = vertices,
            .area = 5 * kR * kR * std::sin(kTheta / 2) * std::cos(kTheta / 2),
            .normal = X_WM.rotation() * Vector3d(1, 0, 0),
            .centroid = X_WM.translation()};
  }

  /* Build a mesh from a number of polygons. */
  PolygonSurfaceMesh<T> MakeMesh(const vector<Polygon> polygons) {
    vector<Vector3<T>> vertices;
    vector<int> face_data;
    for (const auto& polygon : polygons) {
      AddPolygon(polygon, &face_data, &vertices);
    }
    return {move(face_data), move(vertices)};
  }
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(PolygonSurfaceMeshTest, ScalarTypes);

/* Simple confirmation that the default constructor creates a valid, but empty
 mesh. */
TYPED_TEST(PolygonSurfaceMeshTest, DefaultConstructor) {
  using T = TypeParam;
  PolygonSurfaceMesh<T> mesh;
  EXPECT_EQ(mesh.num_vertices(), 0);
  EXPECT_EQ(mesh.num_elements(), 0);
  EXPECT_EQ(mesh.num_faces(), 0);
  EXPECT_EQ(ExtractDoubleOrThrow(mesh.total_area()), 0);
  EXPECT_TRUE(
      CompareMatrices(ExtractDoubleOrThrow(mesh.centroid()), Vector3d::Zero()));
  EXPECT_TRUE(PolygonSurfaceMesh<T>().Equal(PolygonSurfaceMesh<T>()));
}

/* Tests the advanced constructor and the various accessors (including the
 SurfacePolygon API). confirms that it produces the same mesh as
 building a mesh by adding vertices/polygons via the main API. This test relies
 on the AddVertex(), AddPolygon(), and Equal() methods as already tested. */
TYPED_TEST(PolygonSurfaceMeshTest, FullConstructor) {
  using T = TypeParam;

  /* we'll build a mesh consisting of these polygons. */
  vector<Polygon> polygons = {this->MakeIsoscelesTriangle(), this->MakeSquare(),
                              this->MakePentagon()};

  const PolygonSurfaceMesh<T> mesh = this->MakeMesh(polygons);
  const int expected_face_count = static_cast<int>(polygons.size());
  int expected_vert_count = 0;
  for (const auto& polygon : polygons) {
    expected_vert_count += static_cast<int>(polygon.vertices.size());
  }

  EXPECT_EQ(mesh.num_vertices(), expected_vert_count);
  EXPECT_EQ(mesh.num_elements(), expected_face_count);
  EXPECT_EQ(mesh.num_faces(), expected_face_count);
  for (int f = 0; f < expected_face_count; ++f) {
    const auto& face = mesh.element(f);
    const int v_count = static_cast<int>(polygons[f].vertices.size());
    EXPECT_EQ(face.num_vertices(), v_count);
    for (int v = 0; v < v_count; ++v) {
      ASSERT_TRUE(CompareMatrices(mesh.vertex(face.vertex(v)),
                                  polygons[f].vertices[v]));
    }
  }
}

/* This tests centroid computation: both the per-polygon centroid and the
 overall mesh centroid. By using polygons with different numbers of vertices,
 we'll confirm that the per-polygon calculation is correct (as reported by
 element_centroid()) and that they are subsequently combined correctly (as
 reported by centroid()) */
TYPED_TEST(PolygonSurfaceMeshTest, MeshCentroid) {
  using T = TypeParam;

  /* Arbitrary transform so polygons don't have centroids at the origin. */
  const RigidTransformd X_WM{
      AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()}, Vector3d{1, 2, 3}};
  vector<Polygon> polygons = {this->MakeIsoscelesTriangle(X_WM),
                              this->MakeSquare(X_WM), this->MakePentagon(X_WM)};

  double total_area_expected = 0;
  Vector3d scaled_centroid_expected(0, 0, 0);
  /* We're using asserts throughout this for loop because if *one* polygon
   fails they will likely all fail in an unhelpful way; so we'll stop at the
   first. */
  vector<Vector3<T>> vertices;
  vector<int> face_data;
  for (const auto& polygon : polygons) {
    /* Accumulate the polygon into the full mesh. */
    this->AddPolygon(polygon, &face_data, &vertices);
    total_area_expected += polygon.area;
    scaled_centroid_expected += polygon.area * polygon.centroid;
  }
  const PolygonSurfaceMesh<T> mesh(move(face_data), move(vertices));

  ASSERT_TRUE(CompareMatrices(
      mesh.centroid(), scaled_centroid_expected / total_area_expected, 1e-15));
  for (int e = 0; e < mesh.num_elements(); ++e) {
    ASSERT_TRUE(
        CompareMatrices(mesh.element_centroid(e), polygons[e].centroid, 1e-15));
  }
}

TYPED_TEST(PolygonSurfaceMeshTest, Area) {
  using T = TypeParam;

  /* We want to make sure the area is invariant w.r.t. orientation and position
   of the polygon, so we'll apply a non-trivial transform. */
  const RigidTransformd X_WM{
      AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()}, Vector3d{1, 2, 3}};
  const vector<Polygon> polygons = {this->MakeIsoscelesTriangle(X_WM),
                                    this->MakeSquare(X_WM),
                                    this->MakePentagon(X_WM)};

  double expected_total_area = 0;
  vector<Vector3<T>> vertices;
  vector<int> face_data;
  for (const auto& polygon : polygons) {
    /* We're using asserts throughout this for loop because if *one* polygon
     fails they will likely all fail in an unhelpful way; so we'll stop at the
     first. */
    this->AddPolygon(polygon, &face_data, &vertices);
    expected_total_area += polygon.area;
  }
  const PolygonSurfaceMesh<T> mesh(move(face_data), move(vertices));

  ASSERT_NEAR(ExtractDoubleOrThrow(mesh.total_area()), expected_total_area,
              1e-15);
  for (int e = 0; e < mesh.num_elements(); ++e) {
    ASSERT_NEAR(ExtractDoubleOrThrow(mesh.area(e)), polygons[e].area, 1e-15);
  }
}

TYPED_TEST(PolygonSurfaceMeshTest, CalcBoundingBox) {
  using T = TypeParam;

  /* Make sure the box is well-defined in the frame of the vertices by using
   a non-trivial pose. */
  const RigidTransformd X_WM{
      AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()}, Vector3d{1, 2, 3}};
  const vector<Polygon> polygons = {this->MakeIsoscelesTriangle(X_WM),
                                    this->MakeSquare(X_WM),
                                    this->MakePentagon(X_WM)};

  Vector3d min_corner =
      Vector3d::Constant(std::numeric_limits<double>::infinity());
  Vector3d max_corner = -min_corner;

  vector<Vector3<T>> vertices;
  vector<int> face_data;
  for (const auto& polygon : polygons) {
    this->AddPolygon(polygon, &face_data, &vertices);
    for (const auto& v : polygon.vertices) {
      min_corner = min_corner.array().min(v.array());
      max_corner = max_corner.array().max(v.array());
    }
  }
  const PolygonSurfaceMesh<T> mesh(move(face_data), move(vertices));

  const Vector3d center_expected = (min_corner + max_corner) / 2;
  const Vector3d size_expected = max_corner - min_corner;
  const auto [center, size] = mesh.CalcBoundingBox();
  EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(center), center_expected));
  EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(size), size_expected));
}

TYPED_TEST(PolygonSurfaceMeshTest, FaceNormals) {
  using T = TypeParam;

  /* Make sure the face normals computed are in the frame of the vertices by
   using a non-trivial pose. */
  const RigidTransformd X_WM{
      AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()}, Vector3d{1, 2, 3}};
  const vector<Polygon> polygons = {this->MakeIsoscelesTriangle(X_WM),
                                    this->MakeSquare(X_WM),
                                    this->MakePentagon(X_WM)};

  const PolygonSurfaceMesh<T> mesh = this->MakeMesh(polygons);

  for (int p = 0; p < static_cast<int>(polygons.size()); ++p) {
    ASSERT_TRUE(CompareMatrices(ExtractDoubleOrThrow(mesh.face_normal(p)),
                                polygons[p].normal, 1e-15));
  }
}

/* In the case where a face is zero-area, we want to confirm that we get the
 zero vector for a face normal. */
TYPED_TEST(PolygonSurfaceMeshTest, ZeroFaceNormal) {
  using T = TypeParam;

  /* The mesh is a single triangle built on collinear vertices. We leave the
   vertex positions axis aligned in the mesh's frame to *guarantee* getting
   a zero cross product between the triangle edges. Any non-zero bit will
   produce a non-zero normal. */
  const Polygon poly{.name = "line",
                     .vertices = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}},
                     .area = 0,
                     .normal = {0, 0, 0},
                     .centroid = {0, 0, 0}};
  /* Make sure the face normals computed are in the frame of the vertices. */
  const PolygonSurfaceMesh<T> mesh = this->MakeMesh({poly});

  ASSERT_TRUE(
      CompareMatrices(ExtractDoubleOrThrow(mesh.face_normal(0)), poly.normal));
}

/* In the case where a face has collinear vertices, we want to confirm that we
 still get the actual polygon area. */
TYPED_TEST(PolygonSurfaceMeshTest, DegenerateFaceNormal) {
  using T = TypeParam;

  /* We'll create a triangle with one of the edges split by a vertex:

      o
      |\
      | \
      o  \
      |   \
      |    \
      o-----o

   We'll try it on four permutations such that each vertex takes it in turn to
   be the *first* vertex in the polygon. This will show that the code is robust
   to vertex ordering. */
  vector<Vector3d> vertices{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0.5, 0}};
  vector<vector<int>> polygons{
      {0, 1, 2, 3}, {1, 2, 3, 0}, {2, 3, 0, 1}, {3, 0, 1, 2}};
  for (const auto& ordering : polygons) {
    vector<Vector3d> ordered_vertices;
    for (int index : ordering) {
      ordered_vertices.push_back(vertices[index]);
    }
    const Polygon poly{.name = "unused",
                       .vertices = ordered_vertices,
                       .area = 0.5,
                       .normal = {0, 0, 1},
                       .centroid = {1 / 3.0, 1 / 3.0, 0.0}};
    const PolygonSurfaceMesh<T> mesh = this->MakeMesh({poly});

    ASSERT_TRUE(CompareMatrices(ExtractDoubleOrThrow(mesh.face_normal(0)),
                                poly.normal));
    ASSERT_TRUE(CompareMatrices(ExtractDoubleOrThrow(mesh.element_centroid(0)),
                                poly.centroid));
    ASSERT_EQ(mesh.total_area(), poly.area);
  }
}

TYPED_TEST(PolygonSurfaceMeshTest, ReverseFaceWinding) {
  using T = TypeParam;

  PolygonSurfaceMesh<T> mesh =
      this->MakeMesh({this->MakeIsoscelesTriangle(), this->MakeSquare(),
                      this->MakePentagon()});

  /* Create a copy to preserve the original winding to test against. */
  const PolygonSurfaceMesh<T> mesh_copy(mesh);
  mesh.ReverseFaceWinding();

  for (int p = 0; p < mesh.num_elements(); ++p) {
    const SurfacePolygon face = mesh.element(p);
    const SurfacePolygon face_copy = mesh_copy.element(p);
    const int v_count = face.num_vertices();
    for (int i = 0; i < v_count; ++i) {
      ASSERT_EQ(face.vertex(i), face_copy.vertex(v_count - 1 - i));
    }
    ASSERT_TRUE(
        CompareMatrices(mesh.face_normal(p), -mesh_copy.face_normal(p)));
  }
}

TYPED_TEST(PolygonSurfaceMeshTest, TransformVertices) {
  using T = TypeParam;

  /* Create a mesh in the mesh's frame M. */
  PolygonSurfaceMesh<T> mesh_M =
      this->MakeMesh({this->MakeIsoscelesTriangle(), this->MakeSquare(),
                      this->MakePentagon()});

  /* Now create a mesh directly in the world frame with an arbitrary,
   non-trivial pose. */
  const RigidTransformd X_WM{
      AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()}, Vector3d{1, 2, 3}};
  const PolygonSurfaceMesh<T> mesh_W =
      this->MakeMesh({this->MakeIsoscelesTriangle(X_WM), this->MakeSquare(X_WM),
                      this->MakePentagon(X_WM)});

  /* Quick indicator that the two meshes do *not* have the same pose. */
  ASSERT_FALSE(CompareMatrices(mesh_M.vertex(0), mesh_W.vertex(0), 1e-8));

  /* Now mesh_M is also mesh_W. */
  mesh_M.TransformVertices(X_WM.cast<T>());

  ASSERT_NEAR(ExtractDoubleOrThrow(mesh_M.total_area()),
              ExtractDoubleOrThrow(mesh_W.total_area()), 1e-15)
      << "Area should be invariant to rigid vertex transformations";
  for (int v = 0; v < mesh_M.num_vertices(); ++v) {
    ASSERT_TRUE(CompareMatrices(mesh_M.vertex(v), mesh_W.vertex(v), 1e-15));
  }
  for (int p = 0; p < mesh_M.num_elements(); ++p) {
    ASSERT_TRUE(
        CompareMatrices(mesh_M.face_normal(p), mesh_W.face_normal(p), 1e-15));
    ASSERT_TRUE(CompareMatrices(mesh_M.element_centroid(p),
                                mesh_W.element_centroid(p), 1e-15));
  }
  ASSERT_TRUE(CompareMatrices(mesh_M.centroid(), mesh_W.centroid(), 1e-15));
}

TYPED_TEST(PolygonSurfaceMeshTest, CalcGradientVectorOfLinearField) {
  using T = TypeParam;

  /* Create a mesh in the mesh's frame M. */
  PolygonSurfaceMesh<T> mesh_M =
      this->MakeMesh({this->MakeIsoscelesTriangle(), this->MakeSquare(),
                      this->MakePentagon()});
  std::array<T, 3> dummy_values;
  DRAKE_EXPECT_THROWS_MESSAGE(
      mesh_M.CalcGradientVectorOfLinearField(dummy_values, 0),
      "PolygonSurfaceMesh::CalcGradientVectorOfLinearField\\(\\): "
      "PolygonSurfaceMesh does not support this calculation. Defining a "
      "MeshFieldLinear on a PolygonSurfaceMesh requires field gradients to "
      "be provided at construction.");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
