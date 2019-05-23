#include "drake/geometry/proximity/contact_surface_calculator.h"

#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using Eigen::Vector3d;

namespace drake {
namespace geometry {
namespace internal {

// Computes the volume of a tetrahedron given the four vertices that define it.
// The convention is that the first three vertices v0, v1, v2 define a triangle
// with its right-handed normal pointing towards the inside of the tetrahedra.
// The fourth vertex, v3, is on the positive side of the plane defined by v0,
// v1, v2. With this convention, the computed volume will be positive, otherwise
// negative.
double CalcTetrahedronVolume(const Vector3<double>& v0,
                             const Vector3<double>& v1,
                             const Vector3<double>& v2,
                             const Vector3<double>& v3) {
  return (v1 - v0).dot((v2 - v0).cross(v3 - v0)) / 6.0;
}

// Computes the total volume of a TetrahedraMesh by summing up the contribution
// of each tetrahedra.
double CalcTetrahedraMeshVolume(const VolumeMesh<double>& mesh) {
  const std::vector<VolumeVertex<double>>& vertices = mesh.vertices();
  const std::vector<VolumeElement>& tetrahedra = mesh.tetrahedra();
  double volume = 0.0;
  for (const auto& t : tetrahedra) {
    const double tet_volume =
        CalcTetrahedronVolume(vertices[t[0]].r_MV(), vertices[t[1]].r_MV(),
                              vertices[t[2]].r_MV(), vertices[t[3]].r_MV());
    volume += tet_volume;
  }
  return volume;
}

// Tester class with access to private internals of ContactSurfaceCalculator.
template <class T>
class ContactSurfaceCalculatorTester {
 public:
  int IntersectTetWithLevelSet(const std::vector<Vector3<T>>& tet_vertices_N,
                               const Vector4<T>& phi_N,
                               std::vector<SurfaceVertex<T>>* vertices,
                               std::vector<SurfaceFace>* faces) const {
    return ContactSurfaceCalculator<T>::IntersectTetWithLevelSet(
        tet_vertices_N, phi_N, vertices, faces);
  }
};

// Fixture to test the internals of ContactSurfaceCalculator.
// We will test the intersection of a level set with a single regular
// tetrahedron. All cases in the marching tetrahedra algorithm are tested.
class TetrahedronIntersectionTest : public ::testing::Test {
 protected:
  void SetUp() { unit_tet_ = MakeRegularTetrahedron(); }

  // Helper to make a tetrahedron with unit length edges.
  static std::vector<Vector3<double>> MakeRegularTetrahedron() {
    const double face_height = sqrt(3.0) / 2.0;
    const double tet_height = sqrt(6.0) / 3.0;
    std::vector<Vector3<double>> vertices;
    vertices.emplace_back(2. / 3. * face_height, 0.0, 0.0);
    vertices.emplace_back(-1. / 3. * face_height, -0.5, 0.0);
    vertices.emplace_back(-1. / 3. * face_height, 0.5, 0.0);
    vertices.emplace_back(0.0, 0.0, tet_height);
    return vertices;
  }

  // This method computes the right handed normal defined by the three vertices
  // of a triangle in the input argument v.
  static Vector3<double> CalcTriangleNormal(
      const std::vector<SurfaceVertex<double>>& v) {
    return ((v[1].r_MV() - v[0].r_MV()).cross(v[2].r_MV() - v[0].r_MV()))
        .normalized();
  }

  ContactSurfaceCalculator<double> calculator_;
  ContactSurfaceCalculatorTester<double> tester_;
  std::vector<Vector3<double>> unit_tet_;
};

// Verifies we get an empty intersection when all vertices have the same sign.
TEST_F(TetrahedronIntersectionTest, EmptyIntersection) {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;

  // All positive vertices.
  Vector4<double> phi_N = Vector4<double>::Ones();
  EXPECT_EQ(
      tester_.IntersectTetWithLevelSet(unit_tet_, phi_N, &vertices, &faces), 0);

  // All negative vertices.
  phi_N = -Vector4<double>::Ones();
  EXPECT_EQ(
      tester_.IntersectTetWithLevelSet(unit_tet_, phi_N, &vertices, &faces), 0);
}

// Case I of marching tetrahedra: only one of the vertices has a sign different
// from all the other three.
TEST_F(TetrahedronIntersectionTest, CaseI) {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;
  const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();

  const int expected_num_intersections = 3;

  // Computes a unit vector in the direction of vertex "top" from the centroid
  // of the face opposite to "top".
  auto CalcUnitVectorFromBaseToTop = [& unit_tet = this->unit_tet_](int top) {
    const Vector3<double>& to = unit_tet[top];
    // Index to the other three vertices.
    const Vector3<double>& v1 = unit_tet[++top % 4];
    const Vector3<double>& v2 = unit_tet[++top % 4];
    const Vector3<double>& v3 = unit_tet[++top % 4];
    const Vector3<double> from = (v1 + v2 + v3) / 3.0;
    return (to - from).normalized();
  };

  // All vertices are positive but the i-th vertex.
  for (int i = 0; i < 4; ++i) {
    vertices.clear();
    faces.clear();
    Vector4<double> phi_N = Vector4<double>::Ones();
    phi_N[i] = -1.0;
    ASSERT_EQ(
        tester_.IntersectTetWithLevelSet(unit_tet_, phi_N, &vertices, &faces),
        expected_num_intersections);

    ASSERT_EQ(faces.size(), 1);
    const Vector3<int> expected_face(0, 1, 2);
    const Vector3<int> face(faces[0][0], faces[0][1], faces[0][2]);
    EXPECT_EQ(face, expected_face);

    ASSERT_EQ(vertices.size(), 3);

    // The negative sign is due to the fact that the i-th vertex is negative.
    const Vector3<double> expected_normal = -CalcUnitVectorFromBaseToTop(i);
    const Vector3<double> normal = CalcTriangleNormal(vertices);
    EXPECT_TRUE(CompareMatrices(normal, expected_normal, kTolerance));
  }

  // All vertices are negative but the i-th vertex.
  for (int i = 0; i < 4; ++i) {
    vertices.clear();
    faces.clear();
    Vector4<double> phi_N = -Vector4<double>::Ones();
    phi_N[i] = 1.0;
    ASSERT_EQ(
        tester_.IntersectTetWithLevelSet(unit_tet_, phi_N, &vertices, &faces),
        expected_num_intersections);

    ASSERT_EQ(faces.size(), 1);
    const Vector3<int> expected_face(0, 1, 2);
    const Vector3<int> face(faces[0][0], faces[0][1], faces[0][2]);
    EXPECT_EQ(face, expected_face);

    ASSERT_EQ(vertices.size(), 3);

    // The expected normal points in the direction of the i-th vertex.
    const Vector3<double> expected_normal = CalcUnitVectorFromBaseToTop(i);
    const Vector3<double> normal = CalcTriangleNormal(vertices);
    EXPECT_TRUE(CompareMatrices(normal, expected_normal, kTolerance));
  }
}

// Case II of marching tetrahedra: two pairs of vertices with the same sign.
TEST_F(TetrahedronIntersectionTest, CaseII) {
  // This method assumes that two of vertices are positive (thus defining a
  // positive edge) and two of vertices are negative (thus defining a negative
  // edge).
  auto CalcUnitVectorFromNegativeEdgeToPositveEdge =
      [& unit_tet = this->unit_tet_](const Vector4<double>& phi) {
        Vector3<double> from = Vector3<double>::Zero();
        Vector3<double> to = Vector3<double>::Zero();
        for (int i = 0; i < 4; ++i) {
          (phi[i] > 0.0 ? to : from) += unit_tet[i];
        }
        to /= 2.0;
        from /= 2.0;
        return (to - from).normalized();
      };

  auto VerifyCaseII = [&](const Vector4<double>& phi_N) {
    const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();
    const int expected_num_vertices = 5;

    std::vector<SurfaceVertex<double>> vertices;
    std::vector<SurfaceFace> faces;
    ASSERT_EQ(
        tester_.IntersectTetWithLevelSet(unit_tet_, phi_N, &vertices, &faces),
        expected_num_vertices);

    ASSERT_EQ(faces.size(), 4);
    ASSERT_EQ(vertices.size(), 5);

    const Vector3<double> expected_normal =
        CalcUnitVectorFromNegativeEdgeToPositveEdge(phi_N);

    for (int i = 0; i < 4; ++i) {
      const SurfaceFace& t = faces[i];
      std::vector<SurfaceVertex<double>> triangle_vertices = {
          vertices[t[0]], vertices[t[1]], vertices[t[2]]};
      const Vector3<double> normal = CalcTriangleNormal(triangle_vertices);
      EXPECT_TRUE(CompareMatrices(normal, expected_normal, kTolerance));
    }
  };

  // Verify case II for the six different cases.
  VerifyCaseII(Vector4<double>(1.0, 1.0, -1.0, -1.0));
  VerifyCaseII(Vector4<double>(-1.0, -1.0, 1.0, 1.0));
  VerifyCaseII(Vector4<double>(-1.0, 1.0, -1.0, 1.0));
  VerifyCaseII(Vector4<double>(1.0, -1.0, 1.0, -1.0));
  VerifyCaseII(Vector4<double>(1.0, -1.0, -1.0, 1.0));
  VerifyCaseII(Vector4<double>(-1.0, 1.0, 1.0, -1.0));
}

// Fixture to test the intersection of a level set with a unit box.
class BoxPlaneIntersectionTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Generate a partition of the unit cube into 6 tetrahedra.
    std::vector<VolumeVertex<double>> vertices = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}};
    std::vector<VolumeElement> elements = {{0, 1, 3, 7}, {1, 0, 4, 7},
                                           {1, 2, 3, 7}, {2, 1, 5, 7},
                                           {1, 4, 5, 7}, {2, 5, 6, 7}};
    box_B_ = std::make_unique<VolumeMesh<double>>(std::move(elements),
                                                  std::move(vertices));
    half_space_H_ = [](const Vector3<double>& p_HQ) { return p_HQ[2]; };
  }

  double CalcSurfaceArea(const SurfaceMesh<double>& mesh) {
    double area = 0;
    for (SurfaceFaceIndex f(0); f < mesh.num_faces(); ++f) {
      area += mesh.area(f);
    }
    return area;
  }

  // Mesh modeling a box, with its vertex positions expressed in the frame of
  // the box B.
  std::unique_ptr<VolumeMesh<double>> box_B_;

  // A level set function, chosen to be the distance function, for a half space.
  // It is defined as a function φ: ℝ³ → ℝ with the input position vector
  // expressed in the frame H of the half space. Frame H is defined such that Hz
  // normal to the half space pointint into the positive direction (outwards).
  std::function<double(const Vector3<double>&)> half_space_H_;
};

// The box is machine epsilon from making contact with the plane.
TEST_F(BoxPlaneIntersectionTest, ImminentContact) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const double kTolerance = 5.0 * kEpsilon;

  // The box overlaps the plane by kEpsilon. Expect intersection.
  {
    const math::RigidTransformd X_HB =
        Translation3<double>(0.0, 0.0, -kEpsilon);
    const SurfaceMesh<double> contact_surface =
        ContactSurfaceCalculator<double>::CalcZeroLevelSetInMeshDomain(
            *box_B_, half_space_H_, X_HB);
    EXPECT_NEAR(CalcSurfaceArea(contact_surface), 1.0, kTolerance);
  }

  // The box is on top of the plane by kEpsilon. Expect no intersection.
  {
    const math::RigidTransformd X_HB = Translation3<double>(0.0, 0.0, kEpsilon);
    const SurfaceMesh<double> contact_surface =
        ContactSurfaceCalculator<double>::CalcZeroLevelSetInMeshDomain(
            *box_B_, half_space_H_, X_HB);
    EXPECT_NEAR(CalcSurfaceArea(contact_surface), 0.0, kTolerance);
  }
}

// When one of the axes of the box is aligned with the plane normal the contact
// surface simply is a unit square. The contact area however gets discretized
// by a complex mesh of triangles given the non-homogenous tessellation of the
// original box. Therefore verifying that the contact surface has unit area is a
// very good measure of the success of the algorithm.
// We do this for several orientations.
TEST_F(BoxPlaneIntersectionTest, VerifyContactArea) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const double kTolerance = 5.0 * kEpsilon;

  // Notice that the zero in the frame B of the box is at one of the corners,
  // not at the center. Therefore we add one (1.0) to all translations involving
  // a 180 degrees rotation.
  const auto Rx_pi_2 = RollPitchYawd(M_PI_2, 0.0, 0.0);
  const auto Ry_pi_2 = RollPitchYawd(M_PI_2, 0.0, 0.0);
  const auto Rx_pi = RollPitchYawd(M_PI, 0.0, 0.0);
  const auto Ry_pi = RollPitchYawd(M_PI, 0.0, 0.0);
  std::vector<math::RigidTransformd> poses = {
      Translation3<double>(0.0, 0.0, -0.9),
      Translation3<double>(0.0, 0.0, -0.45),
      Translation3<double>(0.0, 0.0, -0.1),
      RigidTransformd(Rx_pi_2, Vector3d(0.0, 0.0, -0.9)),
      RigidTransformd(Rx_pi_2, Vector3d(0.0, 0.0, -0.45)),
      RigidTransformd(Rx_pi_2, Vector3d(0.0, 0.0, -0.1)),
      RigidTransformd(Ry_pi_2, Vector3d(0.0, 0.0, -0.9)),
      RigidTransformd(Ry_pi_2, Vector3d(0.0, 0.0, -0.45)),
      RigidTransformd(Ry_pi_2, Vector3d(0.0, 0.0, -0.1)),
      RigidTransformd(Rx_pi, Vector3d(0.0, 0.0, 1.0 - 0.9)),
      RigidTransformd(Rx_pi, Vector3d(0.0, 0.0, 1.0 - 0.45)),
      RigidTransformd(Rx_pi, Vector3d(0.0, 0.0, 1.0 - 0.1)),
      RigidTransformd(Ry_pi, Vector3d(0.0, 0.0, 1.0 - 0.9)),
      RigidTransformd(Ry_pi, Vector3d(0.0, 0.0, 1.0 - 0.45)),
      RigidTransformd(Ry_pi, Vector3d(0.0, 0.0, 1.0 - 0.1))};

  for (const auto& X_HB : poses) {
    const SurfaceMesh<double> contact_surface =
        ContactSurfaceCalculator<double>::CalcZeroLevelSetInMeshDomain(
            *box_B_, half_space_H_, X_HB);

    EXPECT_NEAR(CalcSurfaceArea(contact_surface), 1.0, kTolerance);
  }
}

// Verify the algorithm returns no intersections when the box is over the plane.
TEST_F(BoxPlaneIntersectionTest, NoIntersection) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const double kTolerance = 5.0 * kEpsilon;

  // Notice that the zero in the frame B of the box is at one of the corners,
  // not at the center. Therefore we add one (1.0) to all translations involving
  // a 180 degrees rotation.
  const auto Rx_pi_2 = RollPitchYawd(M_PI_2, 0.0, 0.0);
  const auto Ry_pi_2 = RollPitchYawd(M_PI_2, 0.0, 0.0);
  const auto Rx_pi = RollPitchYawd(M_PI, 0.0, 0.0);
  const auto Ry_pi = RollPitchYawd(M_PI, 0.0, 0.0);
  std::vector<math::RigidTransformd> poses = {
      Translation3<double>(0.0, 0.0, 0.9),
      Translation3<double>(0.0, 0.0, 0.45),
      Translation3<double>(0.0, 0.0, 0.1),
      RigidTransformd(Rx_pi_2, Vector3d(0.0, 0.0, 0.9)),
      RigidTransformd(Rx_pi_2, Vector3d(0.0, 0.0, 0.45)),
      RigidTransformd(Rx_pi_2, Vector3d(0.0, 0.0, 0.1)),
      RigidTransformd(Ry_pi_2, Vector3d(0.0, 0.0, 0.9)),
      RigidTransformd(Ry_pi_2, Vector3d(0.0, 0.0, 0.45)),
      RigidTransformd(Ry_pi_2, Vector3d(0.0, 0.0, 0.1)),
      RigidTransformd(Rx_pi, Vector3d(0.0, 0.0, 1.0 + 0.9)),
      RigidTransformd(Rx_pi, Vector3d(0.0, 0.0, 1.0 + 0.45)),
      RigidTransformd(Rx_pi, Vector3d(0.0, 0.0, 1.0 + 0.1)),
      RigidTransformd(Ry_pi, Vector3d(0.0, 0.0, 1.0 + 0.9)),
      RigidTransformd(Ry_pi, Vector3d(0.0, 0.0, 1.0 + 0.45)),
      RigidTransformd(Ry_pi, Vector3d(0.0, 0.0, 1.0 + 0.1))};

  for (const auto& X_HB : poses) {
    const SurfaceMesh<double> contact_surface =
        ContactSurfaceCalculator<double>::CalcZeroLevelSetInMeshDomain(
            *box_B_, half_space_H_, X_HB);
    EXPECT_NEAR(CalcSurfaceArea(contact_surface), 0.0, kTolerance);
  }
}

// TODO(amcastro-tri): Add unit test of sphere vs. half space when #11555 lands.

}  // namespace internal.
}  // namespace geometry.
}  // namespace drake.
