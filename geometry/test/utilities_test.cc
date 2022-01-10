#include "drake/geometry/utilities.h"

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using math::RigidTransform;
using math::RollPitchYaw;

GTEST_TEST(GeometryUtilities, CanonicalizeGeometryName) {
  // Confirms that the canonical version of the given name is unchanged.
  auto expect_unchanged = [](const std::string& name) {
    const std::string canonicalized = CanonicalizeStringName(name);
    EXPECT_EQ(name, canonicalized)
        << "Input string '" << name << "' different from the "
        << "canonicalized name '" << canonicalized << "'";
  };

  // Test various names -- including names with internal whitespace.
  for (const std::string& canonical :
       {"name", "with space", "with\ttab", "with\nnewline",
        "with\vvertical tab", "with\fformfeed"}) {
    // Confirms that the given name canonicalizes to the given canonical name.
    auto expect_canonical = [&canonical](const std::string& name) {
      const std::string canonicalized = CanonicalizeStringName(name);
      EXPECT_EQ(canonical, canonicalized)
          << "Input string '" << name << "' canonicalized to '" << canonicalized
          << "' instead of the expected '" << canonical << "'";
    };

    // The canonical name should always come back as itself.
    expect_unchanged(canonical);

    // Characters that *do* get trimmed off.
    for (const std::string& whitespace : {" ", "\t"}) {
      expect_canonical(whitespace + canonical);
      expect_canonical(whitespace + canonical + whitespace);
      expect_canonical(canonical + whitespace);
    }

    // Characters that do *not* get trimmed off.
    // These should be considered a defect in the SDF trimming logic. An issue
    // has been submitted and these tests should be updated when the sdformat
    // trimming logic has been updated.
    for (const std::string& whitespace : {"\n", "\v", "\f"}) {
      expect_unchanged(whitespace + canonical + whitespace);
      expect_unchanged(whitespace + canonical);
      expect_unchanged(canonical + whitespace);
    }
  }
}

GTEST_TEST(GeometryUtilities, RigidTransformConversion) {
  RigidTransform<double> X_AB{
      RollPitchYaw<double>{M_PI / 3, M_PI / 6, M_PI / 7},
      Vector3<double>{1, 2, 3}};

  // Double to double conversion is just a pass through without copying.
  const RigidTransform<double>& X_AB_converted_ref = convert_to_double(X_AB);
  EXPECT_EQ(&X_AB, &X_AB_converted_ref);

  RigidTransform<AutoDiffXd> X_AB_ad(X_AB.cast<AutoDiffXd>());
  RigidTransform<double> X_AB_ad_converted = convert_to_double(X_AB_ad);
  EXPECT_TRUE(
      CompareMatrices(X_AB.GetAsMatrix34(), X_AB_ad_converted.GetAsMatrix34()));
}

GTEST_TEST(GeometryUtilities, MapKeyRange) {
  const std::unordered_map<int, std::string> values{{1, std::string("one")},
                                                    {2, std::string("two")},
                                                    {3, std::string("three")}};
  MapKeyRange<int, std::string> range(&values);
  int count = 0;
  for (int i : range) {
    ++count;
    EXPECT_EQ(values.count(i), 1);
  }
  EXPECT_EQ(count, values.size());

  std::set<int> read_values_set(range.begin(), range.end());
  EXPECT_EQ(read_values_set.size(), values.size());
  for (int i : read_values_set) {
    EXPECT_EQ(values.count(i), 1);
  }

  std::vector<int> read_values_vector(range.begin(), range.end());
  EXPECT_EQ(read_values_vector.size(), values.size());
  for (int i : read_values_vector) {
    EXPECT_EQ(values.count(i), 1);
  }
}

GTEST_TEST(GeometryUtilities, Vector3Conversion) {
  Vector3<double> p_AB{1, 2, 3};

  Vector3<double> p_AB_converted = convert_to_double(p_AB);
  EXPECT_TRUE(CompareMatrices(p_AB, p_AB_converted));
  // Double to double conversion is just a pass through without copying, so
  // we'll compare addresses.
  const Vector3<double>& p_AB_converted_ref = convert_to_double(p_AB);
  EXPECT_EQ(&p_AB, &p_AB_converted_ref);

  Vector3<AutoDiffXd> p_AB_ad(p_AB);
  Vector3<double> X_AB_ad_converted = convert_to_double(p_AB_ad);
  EXPECT_TRUE(CompareMatrices(p_AB, X_AB_ad_converted));
}

GTEST_TEST(ReadObjForConvex, Test1) {
  // Test with triangulate=false.
  // TODO(hongkai.dai): add the test with triangulate=true.
  for (const double scale : {1., 0.5, 1.5}) {
    const auto [vertices, faces, num_faces] = ReadObjForConvex(
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj"), scale,
        false /*triangulate */);
    EXPECT_EQ(vertices->size(), 8);
    EXPECT_EQ(num_faces, 6);
    Eigen::Matrix<double, 8, 3> vertices_expected;
    // clang-format off
  vertices_expected << -1, -1, -1,
                       -1, -1, 1,
                       -1, 1, 1,
                       -1, 1, -1,
                       1, 1, -1,
                       1, -1, -1,
                       1, -1, 1,
                       1, 1, 1;
    // clang-format on
    vertices_expected *= scale;
    for (int i = 0; i < 8; ++i) {
      bool found_match = false;
      for (int j = 0; j < 8; ++j) {
        if (((*vertices)[i] - vertices_expected.row(j).transpose()).norm() <
            1E-6) {
          found_match = true;
          break;
        }
      }
      EXPECT_TRUE(found_match);
    }
    // Each face has 4 vertices, hence the size of faces is num_faces * (1 + 4).
    // The point on the same face will have one coordinate with the same value.
    EXPECT_EQ(faces->size(), num_faces * 5);
    for (int i = 0; i < num_faces; ++i) {
      bool found_coord = false;
      for (int coord = 0; coord < 3; ++coord) {
        bool coord_same_value = true;
        for (int vert = 2; vert <= 4; ++vert) {
          if ((*vertices)[(*faces)[5 * i + vert]](coord) !=
              (*vertices)[(*faces)[5 * i + 1]](coord)) {
            coord_same_value = false;
            break;
          }
        }
        if (coord_same_value) {
          found_coord = true;
          break;
        }
      }
      EXPECT_TRUE(found_coord);
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
