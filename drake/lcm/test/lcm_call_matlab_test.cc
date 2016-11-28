#include <cmath>

#include "gtest/gtest.h"

#include "drake/lcm/lcm_call_matlab.h"

// Note: Unfortunately these really only test whether the code compiles and
// runs... the actual output must currently be verified by human inspection in
// the matlab window.

namespace drake {
namespace lcm {

GTEST_TEST(TestLcmCallMatlab, DispEigenMatrix) {
  Eigen::Matrix2d m;
  m << 1, 2, 3, 4;
  LcmCallMatlab("disp", 0, m);
}

GTEST_TEST(TestLcmCallMatlab, RemoteVarTest) {
  auto m = LcmCallMatlab("eye", 1, 2);
  LcmCallMatlab("disp", 0, m[0]);  // should display eye(2)
}

GTEST_TEST(TestLcmCallMatlab, SimplePlot) {
  int N = 100;

  Eigen::VectorXd time(N), val(N);
  for (int i = 0; i < N; i++) {
    time[i] = 0.01 * i;
    val[i] = sin(2 * M_PI * time[i]);
  }

  LcmCallMatlab("figure", 0, 1);
  auto h = LcmCallMatlab("plot", 1, time, val);
  LcmCallMatlab("set", 0, h[0], "Color", "r");
}

GTEST_TEST(TestLcmCallMatlab, MeshTest) {
  const int N = 25, M = 35;
  Eigen::VectorXd x(N), y(M);
  Eigen::MatrixXd Z(M, N);
  for (int i = 0; i < N; i++) {
    x(i) = -3.0 + 6.0 * i / (N - 1);
  }
  for (int i = 0; i < M; i++) {
    y(i) = -3.0 + 6.0 * i / (M - 1);
  }
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      Z(j, i) = 3 * pow(1 - x(i), 2) * exp(-pow(x(i), 2) - pow(y(j) + 1, 2)) -
                10 * (x(i) / 5 - pow(x(i), 3) - pow(y(j), 5)) *
                    exp(-pow(x(i), 2) - pow(y(j), 2)) -
                1.0 / 3.0 * exp(-pow(x(i) + 1, 2) - pow(y(j), 2));
    }
  }
  LcmCallMatlab("figure", 0, 2);
  LcmCallMatlab("surf", 0, x, y, Z);
}

}  // namespace lcm
}  // namespace drake
