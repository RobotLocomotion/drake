// Copied from (Apache-2.0):
// https://github.com/oxfordcontrol/Clarabel.cpp/blob/main/examples/cpp/example_sdp.cpp
// https://github.com/oxfordcontrol/Clarabel.cpp/blob/main/examples/cpp/utils.h

#include <cmath>
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

int main(void) {
  // SDP Example
  int n = 3;
  int nvec = (n * (n + 1)) >> 1;

  // 6 x 6 zero matrix
  Eigen::SparseMatrix<double> P =
      Eigen::MatrixXd::Zero(nvec, nvec).sparseView();
  P.makeCompressed();

  Eigen::Vector<double, 6> c{1, 0, 1, 0, 0, 1};

  Eigen::MatrixXd A_dense(7, 6);
  const double s = M_SQRT2;
  // clang-format off
  A_dense <<
      -1, 0,  0,  0,  0,   0,
      0,  -s, 0,  0,  0,   0,
      0,  0,  -1, 0,  0,   0,
      0,  0,  0,  -s, 0,   0,
      0,  0,  0,  0,  -s,  0,
      0,  0,  0,  0,  0,  -1,
      1,  4,  3,  8,  10,  6;
  // clang-format on

  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  A.makeCompressed();

  Eigen::Vector<double, 7> b = {0, 0, 0, 0, 0, 0, 1};

  std::vector<clarabel::SupportedConeT<double>> cones{
      clarabel::PSDTriangleConeT<double>(n),
      clarabel::ZeroConeT<double>(1),
  };

  // Settings
  clarabel::DefaultSettings<double> settings =
      clarabel::DefaultSettings<double>::default_settings();

  // Build solver
  clarabel::DefaultSolver<double> solver(P, c, A, b, cones, settings);

  // Solve
  solver.solve();

  // Get solution
  clarabel::DefaultSolution<double> solution = solver.solution();
  print_solution(solution);

  return 0;
}
