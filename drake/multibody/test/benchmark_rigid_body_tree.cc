#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/test/measure_execution.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient_util.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Matrix3Xd;
using Eigen::Matrix;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::default_random_engine;
using std::endl;
using std::make_pair;
using std::map;
using std::pair;
using std::string;
using std::uniform_real_distribution;
using std::vector;

namespace drake {

using common::test::MeasureExecutionTime;
using math::autoDiffToGradientMatrix;
using math::autoDiffToValueMatrix;
using math::gradientMatrixToAutoDiff;

namespace multibody {
namespace {

typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;
typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;

template <int Rows, int Cols>
void printMatrix(const MatrixBase<Matrix<double, Rows, Cols>>& mat) {
  cout << mat << endl;
}

template <int Rows, int Cols, typename DerType>
void printMatrix(
    const MatrixBase<Matrix<AutoDiffScalar<DerType>, Rows, Cols>>& mat) {
  cout << autoDiffToValueMatrix(mat) << endl;
  cout << autoDiffToGradientMatrix(mat) << endl;
}

template <typename Scalar>
void Scenario1(const RigidBodyTree<double>& model,
               // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
               KinematicsCache<Scalar>& cache,
               const vector<Matrix<Scalar, Dynamic, 1>>& qs,
               const map<int, Matrix3Xd>& body_fixed_points) {
  default_random_engine generator;
  uniform_real_distribution<> uniform(0, 1);

  for (const auto& q : qs) {
    cache.initialize(q);
    model.doKinematics(cache, false);
    for (const auto& pair : body_fixed_points) {
      auto J = model.transformPointsJacobian(cache, pair.second, pair.first, 0,
                                             false);
      if (uniform(generator) < 1e-15) {
        // print with some probability to avoid optimizing away
        printMatrix<decltype(J)::RowsAtCompileTime,
                    decltype(J)::ColsAtCompileTime>(
            J);  // MSVC 2013 can't infer rows and cols (ICE)
      }
    }
  }
}

template <typename Scalar>
void Scenario2(
    const RigidBodyTree<double>& model,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache,
    const vector<pair<Matrix<Scalar, Dynamic, 1>, Matrix<Scalar, Dynamic, 1>>>&
        states) {
  default_random_engine generator;
  uniform_real_distribution<> uniform(0, 1);

  const typename RigidBodyTree<Scalar>::BodyToWrenchMap no_external_wrenches;
  for (const auto& state : states) {
    cache.initialize(state.first, state.second);
    model.doKinematics(cache, true);
    auto H = model.massMatrix(cache);
    auto C = model.dynamicsBiasTerm(cache, no_external_wrenches);
    if (uniform(generator) <
        1e-15) {  // print with some probability to avoid optimizing away
      printMatrix<decltype(H)::RowsAtCompileTime,
                  decltype(H)::ColsAtCompileTime>(
          H);  // MSVC 2013 can't infer rows and cols (ICE)
      printMatrix<decltype(C)::RowsAtCompileTime,
                  decltype(C)::ColsAtCompileTime>(
          C);  // MSVC 2013 can't infer rows and cols (ICE)
    }
  }
}

void TestScenario1(const RigidBodyTree<double>& model) {
  int ntests = 1000;

  vector<VectorXd> qs_double;
  vector<Matrix<AutoDiffFixedMaxSize, Dynamic, 1>> qs_autodiff_fixed;
  vector<Matrix<AutoDiffDynamicSize, Dynamic, 1>> qs_autodiff_dynamic;
  default_random_engine generator;

  for (int i = 0; i < ntests; i++) {
    auto q = model.getRandomConfiguration(generator);
    qs_double.push_back(q);

    MatrixXd grad = MatrixXd::Identity(model.get_num_positions(),
                                       model.get_num_positions());

    auto q_autodiff_fixed = q.cast<AutoDiffFixedMaxSize>().eval();
    gradientMatrixToAutoDiff(grad, q_autodiff_fixed);
    qs_autodiff_fixed.push_back(q_autodiff_fixed);

    auto q_autodiff_dynamic = q.cast<AutoDiffDynamicSize>().eval();
    gradientMatrixToAutoDiff(grad, q_autodiff_dynamic);
    qs_autodiff_dynamic.push_back(q_autodiff_dynamic);
  }

  map<int, Matrix3Xd> body_fixed_points;
  int npoints_feet = 4;
  int npoints_hands = 1;
  int npoints_head = 1;
  vector<string> sides{"l", "r"};
  for (const auto& side : sides) {
    int hand_id = model.FindBodyIndex(side + "_hand");
    body_fixed_points.insert(
        make_pair(hand_id, Matrix3Xd::Random(3, npoints_hands)));

    int foot_id = model.FindBodyIndex(side + "_foot");
    body_fixed_points.insert(
        make_pair(foot_id, Matrix3Xd::Random(3, npoints_feet)));
  }
  int head_id = model.FindBodyIndex("head");
  body_fixed_points.insert(
      make_pair(head_id, Matrix3Xd::Random(3, npoints_head)));

  auto cache_double = model.CreateKinematicsCache();
  auto cache_autodiff_fixed =
      model.CreateKinematicsCacheWithType<AutoDiffFixedMaxSize>();
  auto cache_autodiff_dynamic =
      model.CreateKinematicsCacheWithType<AutoDiffDynamicSize>();

  cout << "scenario 1:" << endl;
  cout << "no gradients: "
       << MeasureExecutionTime(Scenario1<double>, model, cache_double,
                               qs_double, body_fixed_points) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff fixed max size: "
       << MeasureExecutionTime(Scenario1<AutoDiffFixedMaxSize>, model,
                               cache_autodiff_fixed, qs_autodiff_fixed,
                               body_fixed_points) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff dynamic size: "
       << MeasureExecutionTime(Scenario1<AutoDiffDynamicSize>, model,
                               cache_autodiff_dynamic, qs_autodiff_dynamic,
                               body_fixed_points) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << endl;
}

void TestScenario2(const RigidBodyTree<double>& model) {
  int ntests = 1000;

  vector<pair<VectorXd, VectorXd>> states_double;
  vector<pair<Matrix<AutoDiffFixedMaxSize, Dynamic, 1>,
              Matrix<AutoDiffFixedMaxSize, Dynamic, 1>>> states_autodiff_fixed;
  vector<pair<Matrix<AutoDiffDynamicSize, Dynamic, 1>,
              Matrix<AutoDiffDynamicSize, Dynamic, 1>>> states_autodiff_dynamic;
  default_random_engine generator;

  for (int i = 0; i < ntests; i++) {
    VectorXd q = model.getRandomConfiguration(generator);
    VectorXd v = VectorXd::Random(model.get_num_velocities());
    VectorXd x(q.rows() + v.rows());
    x << q, v;
    states_double.push_back(make_pair(q, v));

    MatrixXd grad = MatrixXd::Identity(x.size(), x.size());

    auto x_autodiff_fixed = x.cast<AutoDiffFixedMaxSize>().eval();
    gradientMatrixToAutoDiff(grad, x_autodiff_fixed);
    Matrix<AutoDiffFixedMaxSize, Dynamic, 1> q_autodiff_fixed =
        x_autodiff_fixed.topRows(model.get_num_positions());
    Matrix<AutoDiffFixedMaxSize, Dynamic, 1> v_autodiff_fixed =
        x_autodiff_fixed.bottomRows(model.get_num_velocities());
    states_autodiff_fixed.push_back(
        make_pair(q_autodiff_fixed, v_autodiff_fixed));

    auto x_autodiff_dynamic = x.cast<AutoDiffDynamicSize>().eval();
    gradientMatrixToAutoDiff(grad, x_autodiff_dynamic);
    Matrix<AutoDiffDynamicSize, Dynamic, 1> q_autodiff_dynamic =
        x_autodiff_dynamic.topRows(model.get_num_positions());
    Matrix<AutoDiffDynamicSize, Dynamic, 1> v_autodiff_dynamic =
        x_autodiff_dynamic.bottomRows(model.get_num_velocities());
    states_autodiff_dynamic.push_back(
        make_pair(q_autodiff_dynamic, v_autodiff_dynamic));
  }

  auto cache_double = model.CreateKinematicsCache();
  auto cache_autodiff_fixed =
      model.CreateKinematicsCacheWithType<AutoDiffFixedMaxSize>();
  auto cache_autodiff_dynamic =
      model.CreateKinematicsCacheWithType<AutoDiffDynamicSize>();

  cout << "scenario 2:" << endl;
  cout << "no gradients: "
       << MeasureExecutionTime(Scenario2<double>, model, cache_double,
                               states_double) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff fixed max size: "
       << MeasureExecutionTime(Scenario2<AutoDiffFixedMaxSize>, model,
                               cache_autodiff_fixed, states_autodiff_fixed) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff dynamic size: "
       << MeasureExecutionTime(Scenario2<AutoDiffDynamicSize>, model,
                               cache_autodiff_dynamic,
                               states_autodiff_dynamic) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << endl;
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/Atlas/urdf/atlas_minimal_contact.urdf",
      drake::multibody::joints::kRollPitchYaw, tree.get());

  drake::multibody::TestScenario1(*tree);
  drake::multibody::TestScenario2(*tree);

  return 0;
}
