#include "drake/multibody/fem/mpm-dev/particles_to_bgeo.h"

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

// Write to file and read by from file.
GTEST_TEST(ParticlesToBgeoTest, RoundTrip) {
  const filesystem::path temp_dir = temp_directory();
  const auto file = temp_dir / "test.bgeo";
  std::vector<Vector3<double>> q_expected, v_expected;
  std::vector<double> m_expected;
  q_expected.emplace_back(1, 2, 3);
  v_expected.emplace_back(4, 5, 6);
  m_expected.emplace_back(7);
  WriteParticlesToBgeo(file.string(), q_expected, v_expected, m_expected);

  std::vector<Vector3<double>> q, v;
  std::vector<double> m;
  ReadParticlesFromBgeo(file.string(), &q, &v, &m);

  ASSERT_EQ(q_expected.size(), q.size());
  for (size_t i = 0; i < q.size(); ++i) {
    EXPECT_TRUE(CompareMatrices(q_expected[i], q[i],
                                std::numeric_limits<float>::epsilon()));
  }
  ASSERT_EQ(v_expected.size(), v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_TRUE(CompareMatrices(v_expected[i], v[i],
                                std::numeric_limits<float>::epsilon()));
  }
  ASSERT_EQ(m_expected.size(), m.size());
  for (size_t i = 0; i < m.size(); ++i) {
    EXPECT_EQ(m_expected[i], m[i]);
  }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
