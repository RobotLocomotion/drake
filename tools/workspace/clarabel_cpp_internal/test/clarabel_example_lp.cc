// Copied from (Apache-2.0):
// https://github.com/oxfordcontrol/Clarabel.cpp/blob/main/examples/cpp/example_lp.cpp
// https://github.com/oxfordcontrol/Clarabel.cpp/blob/main/examples/cpp/utils.h

#include <cstddef>
#include <cstdio>
#include <vector>

#include <Clarabel>
#include <Eigen/Eigen>

namespace {

void print_array(const Eigen::Ref<const Eigen::VectorXd>& vec) {
  printf("[");
  size_t n = vec.size();
  for (size_t i = 0; i < n; i++) {
    printf("%.10f", vec.data()[i]);
    if (i < n - 1) {
      printf(", ");
    }
  }
  printf("]\n");
}

void print_solution(const clarabel::DefaultSolution<double>& solution) {
  printf("Solution (x)\t = ");
  print_array(solution.x);
  printf("Multipliers (z)\t = ");
  print_array(solution.z);
  printf("Slacks (s)\t = ");
  print_array(solution.s);
}

}  // namespace

int main() {
  Eigen::MatrixXd P_dense = Eigen::MatrixXd::Zero(2, 2);
  Eigen::SparseMatrix<double> P = P_dense.sparseView();
  P.makeCompressed();

  Eigen::Vector<double, 2> q = {1.0, -1.0};

  // a 2-d box constraint, separated into 4 inequalities.
  // A = [I; -I]
  Eigen::MatrixXd A_dense(4, 2);
  A_dense << 1., 0., 0., 1., -1., 0., 0., -1.;

  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  A.makeCompressed();

  Eigen::Vector<double, 4> b = {1.0, 1.0, 1.0, 1.0};

  std::vector<clarabel::SupportedConeT<double>> cones{
      clarabel::NonnegativeConeT<double>(4),
      // {.tag = SupportedConeT<double>::Tag::NonnegativeConeT,
      // .nonnegative_cone_t = {._0 = 4 }}
  };

  // Settings
  clarabel::DefaultSettings<double> settings =
      clarabel::DefaultSettingsBuilder<double>::default_settings()
          .equilibrate_enable(true)
          .equilibrate_max_iter(50)
          .build();

  // Build solver
  clarabel::DefaultSolver<double> solver(P, q, A, b, cones, settings);

  // Solve
  solver.solve();

  // Get solution
  clarabel::DefaultSolution<double> solution = solver.solution();
  print_solution(solution);

  return 0;
}
