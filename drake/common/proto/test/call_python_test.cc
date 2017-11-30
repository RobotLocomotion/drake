#include "drake/common/proto/call_python.h"

#include <cmath>

#include <gtest/gtest.h>

// TODO(eric.cousineau): Instrument client to verify output (and make this a
// unittest).

namespace drake {
namespace common {

GTEST_TEST(TestCallPython, DispStr) {
  CallPython("print", "Hello");
  CallPython("disp", "World");
}

GTEST_TEST(TestCallPython, CheckKwargs) {
  // Ensure that we can create dict with Kwargs (MATLAB-style of arguments).
  auto out = CallPython("dict", ToPythonKwargs("a", 2, "b", "hello"));
  CallPython("print", out);
}

GTEST_TEST(TestCallPython, SetVar) {
  CallPython("setvar", "example_var", "Howdy");
  // Execute code printing the value.
  CallPython("eval", "print(example_var)");
  // Print a letter.
  CallPython("print", CallPython("eval", "example_var")[2]);
  // Print all variables avaiable.
  CallPython("print", CallPython("locals").attr("keys")());
}

GTEST_TEST(TestCallPython, DispEigenMatrix) {
  Eigen::Matrix2d m;
  m << 1, 2, 3, 4;
  CallPython("print", m);

  Eigen::Matrix<bool, 2, 2> b;
  b << true, false, true, false;
  CallPython("print", b);
}

GTEST_TEST(TestCallPython, RemoteVarTest) {
  auto magic = CallPython("magic", 3);
  // N.B. `var.slice(x, y, ...)` and `var[x][y]...` should be interchangeable if
  // you are dealing with NumPy matrices; however, `slice(...)` will allow
  // strings to be specified for slices (e.g. ":", ":-2", "::-1").
  CallPython("print", magic);
  CallPython("print", "element(0,0) is ");
  CallPython("print", magic[0][0]);
  CallPython("print", "element(2,1) is ");
  // Show using both `operator[]` and `.slice`.
  CallPython("print", magic[2][1]);
  CallPython("print", magic.slice(2, 1));
  CallPython("print", "elements ([0, 2]) are");
  CallPython("print", magic.slice(Eigen::Vector2i(0, 2)));
  CallPython("print", "row 2 is ");
  CallPython("print", magic.slice(2, ":"));
  CallPython("print", "rows [0, 1] are");
  CallPython("print", magic[Eigen::VectorXi::LinSpaced(2, 0, 1)]);

  CallPython("print", "row 1 (accessed via logicals) is");
  CallPython("print", magic.slice(Vector3<bool>(false, true, false), ":"));

  CallPython("print", "Third column should now be [1, 2, 3]: ");
  magic.slice(":", 2) = Eigen::Vector3d(1, 2, 3);
  CallPython("print", magic);
}

GTEST_TEST(TestCallPython, Plot2d) {
  const int N = 100;

  Eigen::VectorXd time(N), val(N);
  for (int i = 0; i < N; i++) {
    time[i] = 0.01 * i;
    val[i] = sin(2 * M_PI * time[i]);
  }

  CallPython("print", "Plotting a sine wave.");
  CallPython("figure", 1);
  CallPython("plot", time, val);
  // Send variables in different ways.
  CallPython("locals")["val"] = val;
  CallPython("locals").attr("update")(ToPythonKwargs("time", time));
  // Check usage.
  CallPython("eval", "print(len(val) + len(time))");
}

GTEST_TEST(TestCallPython, Plot3d) {
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
  CallPython("print", "Plotting a simple 3D surface");
  CallPython("figure", 2);
  CallPython("surf", x, y, Z);
  // Send variables.
  CallPython("setvars", "x", x, "y", y, "Z", Z);
  CallPython("eval", "print(len(x) + len(y) + len(Z))");
}

}  // namespace common
}  // namespace drake
