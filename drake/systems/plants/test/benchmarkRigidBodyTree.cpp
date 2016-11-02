#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/test/measure_execution.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Matrix3Xd;
using Eigen::Matrix;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::common::test::MeasureExecutionTime;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using std::cout;
using std::default_random_engine;
using std::endl;
using std::make_pair;
using std::map;
using std::pair;
using std::string;
using std::uniform_real_distribution;
using std::vector;

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
void scenario1(const RigidBodyTree& model,
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
void scenario2(
    const RigidBodyTree& model,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache,
    const vector<pair<Matrix<Scalar, Dynamic, 1>, Matrix<Scalar, Dynamic, 1>>>&
        states) {
  default_random_engine generator;
  uniform_real_distribution<> uniform(0, 1);

  const RigidBodyTree::BodyToWrenchMap<Scalar> no_external_wrenches;
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

void testScenario1(const RigidBodyTree& model) {
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

  KinematicsCache<double> cache_double(model.bodies);
  KinematicsCache<AutoDiffFixedMaxSize> cache_autodiff_fixed(model.bodies);
  KinematicsCache<AutoDiffDynamicSize> cache_autodiff_dynamic(model.bodies);

  cout << "scenario 1:" << endl;
  cout << "no gradients: "
       << MeasureExecutionTime(scenario1<double>, model, cache_double,
                               qs_double, body_fixed_points) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff fixed max size: "
       << MeasureExecutionTime(scenario1<AutoDiffFixedMaxSize>, model,
                               cache_autodiff_fixed, qs_autodiff_fixed,
                               body_fixed_points) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff dynamic size: "
       << MeasureExecutionTime(scenario1<AutoDiffDynamicSize>, model,
                               cache_autodiff_dynamic, qs_autodiff_dynamic,
                               body_fixed_points) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << endl;
}

void testScenario2(const RigidBodyTree& model) {
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

  KinematicsCache<double> cache_double(model.bodies);
  KinematicsCache<AutoDiffFixedMaxSize> cache_autodiff_fixed(model.bodies);
  KinematicsCache<AutoDiffDynamicSize> cache_autodiff_dynamic(model.bodies);

  cout << "scenario 2:" << endl;
  cout << "no gradients: "
       << MeasureExecutionTime(scenario2<double>, model, cache_double,
                               states_double) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff fixed max size: "
       << MeasureExecutionTime(scenario2<AutoDiffFixedMaxSize>, model,
                               cache_autodiff_fixed, states_autodiff_fixed) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << "autodiff dynamic size: "
       << MeasureExecutionTime(scenario2<AutoDiffDynamicSize>, model,
                               cache_autodiff_dynamic,
                               states_autodiff_dynamic) /
              static_cast<double>(ntests)
       << " s" << endl;
  cout << endl;
}

int main() {
  RigidBodyTree model("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  testScenario1(model);
  testScenario2(model);

  return 0;
}
