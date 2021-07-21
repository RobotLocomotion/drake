#include "drake/geometry/optimization/convex_set.h"

#include <gtest/gtest.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;

class MutablePoint final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MutablePoint)

  explicit MutablePoint(const Eigen::Ref<const Eigen::VectorXd>& x)
      : ConvexSet(&ConvexSetCloner<MutablePoint>, x.size()), x_{x} {}
  ~MutablePoint() final {}

  const Eigen::VectorXd& x() const { return x_; }
  Eigen::VectorXd& x() { return x_; }

 private:
  // These are unused, and (effectively) unimplemented.
  bool DoIsBounded() const final { return true; }

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final {
      return is_approx_equal_abstol(x, x_, tol);
  }

  void DoAddPointInSetConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&) const final {
    throw std::runtime_error("Not implemented");
  }

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final {
    throw std::runtime_error("Not implemented");
  }

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final {
    throw std::runtime_error("Not implemented");
  }

  Eigen::VectorXd x_;
};

GTEST_TEST(ConvexSetsTest, BasicTest) {
  ConvexSets sets;

  const ConvexSet& a =
      *sets.emplace_back(MutablePoint(Vector2d{1., 2.}));
  const Vector3d b_point{3., 4., 5.};
  std::unique_ptr<MutablePoint> b_original =
      std::make_unique<MutablePoint>(b_point);
  MutablePoint* b_pointer = b_original.get();
  const ConvexSet& b = *sets.emplace_back(std::move(b_original));

  EXPECT_EQ(a.ambient_dimension(), 2);
  EXPECT_EQ(b.ambient_dimension(), 3);

  EXPECT_EQ(sets.size(), 2);
  EXPECT_EQ(sets[0]->ambient_dimension(), 2);
  EXPECT_EQ(sets[1]->ambient_dimension(), 3);

  // Confirm that a const reference to the container provides only const access
  // to the set.
  const ConvexSets& const_sets = sets;
  static_assert(std::is_same_v<const ConvexSet&, decltype(*const_sets[0])>);

  // Confirm that I can move sets without copying the underlying data.
  // Note: jwnimmer-tri argued that this should not be a strong requirement.
  // Derived ConvexSets with substantial memory footprint could implement
  // Clone() using a shared_ptr on their data.  It may be fine to remove this if
  // a different pattern requires it.
  ConvexSets moved = std::move(sets);
  EXPECT_EQ(moved.size(), 2);
  EXPECT_EQ(moved[0]->ambient_dimension(), 2);
  EXPECT_EQ(moved[1]->ambient_dimension(), 3);
  EXPECT_TRUE(moved[1]->PointInSet(b_point));
  const Vector3d new_point{6., 7., 8.};
  EXPECT_FALSE(moved[1]->PointInSet(new_point));
  b_pointer->x() = new_point;
  EXPECT_TRUE(moved[1]->PointInSet(new_point));
}
}  // namespace

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
