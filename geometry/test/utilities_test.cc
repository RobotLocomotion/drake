#include "drake/geometry/utilities.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

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

GTEST_TEST(GeometryUtilities, IsometryConversion) {
  Isometry3<double> X_AB = Isometry3<double>::Identity();
  X_AB.translation() << 1, 2, 3;
  // NOTE: Not a valid transform; we're just looking for unique values.
  X_AB.linear() << 10, 20, 30, 40, 50, 60, 70, 80, 90;
  X_AB.makeAffine();

  Isometry3<double> X_AB_converted = convert(X_AB);
  EXPECT_TRUE(CompareMatrices(X_AB.matrix(), X_AB_converted.matrix()));

  Isometry3<AutoDiffXd> X_AB_ad(X_AB);
  Isometry3<double> X_AB_ad_converted = convert(X_AB_ad);
  EXPECT_TRUE(CompareMatrices(X_AB.matrix(), X_AB_ad_converted.matrix()));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
