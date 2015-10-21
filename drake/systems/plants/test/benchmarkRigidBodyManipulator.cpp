#include "RigidBodyManipulator.h"
#include "testUtil.h"
#include <cmath>

using namespace std;
using namespace Eigen;

typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;
typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;

default_random_engine generator;
uniform_real_distribution<> uniform(0, 1);

template <int Rows, int Cols>
void printMatrix(const MatrixBase<Matrix<double, Rows, Cols>>& mat) {
  cout << mat << endl;
};

template <int Rows, int Cols, typename DerType>
void printMatrix(const MatrixBase<Matrix<AutoDiffScalar<DerType>, Rows, Cols>>& mat) {
  cout << autoDiffToValueMatrix(mat) << endl;
  cout << autoDiffToGradientMatrix(mat) << endl;
};

template <typename Scalar>
void scenario1(const RigidBodyManipulator &model, KinematicsCache<Scalar>& cache, const vector<Matrix<Scalar, Dynamic, 1>>& qs, const map<int, Matrix3Xd>& body_fixed_points) {
  for (const auto& q : qs) {
    cache.initialize(q);
    model.doKinematics(cache, false);
    for (const auto& pair : body_fixed_points) {
      auto J = model.forwardKinJacobian(cache, pair.second, pair.first, 0, 2, false, cache.getGradientOrder());
      if (uniform(generator) < 1e-15) {
        printMatrix(J.value()); // print with some probability to avoid optimizing away
        if (cache.getGradientOrder() > 0) {
          printMatrix(J.gradient().value());
        }
      }
    }
  }
}

template <typename Scalar>
void scenario2(const RigidBodyManipulator &model, KinematicsCache<Scalar>& cache, const vector<pair<Matrix<Scalar, Dynamic, 1>, Matrix<Scalar, Dynamic, 1>>>& states) {
  const eigen_aligned_unordered_map<RigidBody const *, GradientVar<Scalar, TWIST_SIZE, 1> > f_ext;
  for (const auto& state : states) {
    cache.initialize(state.first, state.second);
    model.doKinematics(cache, true);
    auto H = model.massMatrix(cache, cache.getGradientOrder());
    auto C = model.dynamicsBiasTerm(cache, f_ext, cache.getGradientOrder());
    if (uniform(generator) < 1e-15) { // print with some probability to avoid optimizing away
      printMatrix(H.value());
      printMatrix(C.value());
      if (cache.getGradientOrder() > 0) {
        printMatrix(H.gradient().value());
        printMatrix(C.gradient().value());
      }
    }
  }
}

void testScenario1(const RigidBodyManipulator& model) {

  int ntests = 1000;

  vector<VectorXd> qs_double;
  vector<Matrix<AutoDiffFixedMaxSize, Dynamic, 1>> qs_autodiff_fixed;
  vector<Matrix<AutoDiffDynamicSize, Dynamic, 1>> qs_autodiff_dynamic;

  for (int i = 0; i < ntests; i++) {
    auto q = VectorXd::Random(model.num_positions).eval();
    qs_double.push_back(q);

    MatrixXd grad = MatrixXd::Identity(model.num_positions, model.num_positions);

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
  vector<string> sides {"l", "r"};
  for (const auto& side : sides) {
    int hand_id = model.findLinkId(side + "_hand");
    body_fixed_points.insert(make_pair(hand_id, Matrix3Xd::Random(3, npoints_hands)));

    int foot_id = model.findLinkId(side + "_foot");
    body_fixed_points.insert(make_pair(foot_id, Matrix3Xd::Random(3, npoints_feet)));
  }
  int head_id = model.findLinkId("head");
  body_fixed_points.insert(make_pair(head_id, Matrix3Xd::Random(3, npoints_head)));

  KinematicsCache<double> cache_double_no_gradients(model.bodies, 0);
  KinematicsCache<double> cache_double_gradients(model.bodies, 1);
  KinematicsCache<AutoDiffFixedMaxSize> cache_autodiff_fixed(model.bodies, 0);
  KinematicsCache<AutoDiffDynamicSize> cache_autodiff_dynamic(model.bodies, 0);

  cout << "scenario 1:" << endl;
  cout << "no gradients: " << measure<>::execution(scenario1<double>, model, cache_double_no_gradients, qs_double, body_fixed_points) / static_cast<double>(ntests) << " ms" << endl;
  cout << "user gradients: " << measure<>::execution(scenario1<double>, model, cache_double_gradients, qs_double, body_fixed_points) / static_cast<double>(ntests) << " ms" << endl;
  cout << "autodiff fixed max size: " << measure<>::execution(scenario1<AutoDiffFixedMaxSize>, model, cache_autodiff_fixed, qs_autodiff_fixed, body_fixed_points) / static_cast<double>(ntests) << " ms" << endl;
  cout << "autodiff dynamic size: " << measure<>::execution(scenario1<AutoDiffDynamicSize>, model, cache_autodiff_dynamic, qs_autodiff_dynamic, body_fixed_points) / static_cast<double>(ntests) << " ms" << endl;
  cout << endl;
}


void testScenario2(const RigidBodyManipulator& model) {
  int ntests = 1000;

  vector<pair<VectorXd, VectorXd>> states_double;
  vector<pair<Matrix<AutoDiffFixedMaxSize, Dynamic, 1>, Matrix<AutoDiffFixedMaxSize, Dynamic, 1>>> states_autodiff_fixed;
  vector<pair<Matrix<AutoDiffDynamicSize, Dynamic, 1>, Matrix<AutoDiffDynamicSize, Dynamic, 1>>> states_autodiff_dynamic;

  for (int i = 0; i < ntests; i++) {
    auto x = VectorXd::Random(model.num_positions + model.num_velocities).eval();
    VectorXd q = x.topRows(model.num_positions);
    VectorXd v = x.bottomRows(model.num_velocities);
    states_double.push_back(make_pair(q, v));

    MatrixXd grad = MatrixXd::Identity(x.size(), x.size());

    auto x_autodiff_fixed = x.cast<AutoDiffFixedMaxSize>().eval();
    gradientMatrixToAutoDiff(grad, x_autodiff_fixed);
    Matrix<AutoDiffFixedMaxSize, Dynamic, 1> q_autodiff_fixed = x_autodiff_fixed.topRows(model.num_positions);
    Matrix<AutoDiffFixedMaxSize, Dynamic, 1> v_autodiff_fixed = x_autodiff_fixed.bottomRows(model.num_velocities);
    states_autodiff_fixed.push_back(make_pair(q_autodiff_fixed, v_autodiff_fixed));

    auto x_autodiff_dynamic = x.cast<AutoDiffDynamicSize>().eval();
    gradientMatrixToAutoDiff(grad, x_autodiff_dynamic);
    Matrix<AutoDiffDynamicSize, Dynamic, 1> q_autodiff_dynamic = x_autodiff_dynamic.topRows(model.num_positions);
    Matrix<AutoDiffDynamicSize, Dynamic, 1> v_autodiff_dynamic = x_autodiff_dynamic.bottomRows(model.num_velocities);
    states_autodiff_dynamic.push_back(make_pair(q_autodiff_dynamic, v_autodiff_dynamic));
  }

  KinematicsCache<double> cache_double_no_gradients(model.bodies, 0);
  KinematicsCache<double> cache_double_gradients(model.bodies, 1);
  KinematicsCache<AutoDiffFixedMaxSize> cache_autodiff_fixed(model.bodies, 0);
  KinematicsCache<AutoDiffDynamicSize> cache_autodiff_dynamic(model.bodies, 0);

  cout << "scenario 2:" << endl;
  cout << "no gradients: " << measure<>::execution(scenario2<double>, model, cache_double_no_gradients, states_double) / static_cast<double>(ntests) << " ms" << endl;
  cout << "user gradients: " << measure<>::execution(scenario2<double>, model, cache_double_gradients, states_double) / static_cast<double>(ntests) << " ms" << endl;
  cout << "autodiff fixed max size: " << measure<>::execution(scenario2<AutoDiffFixedMaxSize>, model, cache_autodiff_fixed, states_autodiff_fixed) / static_cast<double>(ntests) << " ms" << endl;
  cout << "autodiff dynamic size: " << measure<>::execution(scenario2<AutoDiffDynamicSize>, model, cache_autodiff_dynamic, states_autodiff_dynamic) / static_cast<double>(ntests) << " ms" << endl;
  cout << endl;
}

int main() {
  RigidBodyManipulator model("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  testScenario1(model);
  testScenario2(model);

  return 0;
}