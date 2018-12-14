#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/proto/call_python.h"

// N.B. The following flags are used for testing only, and are not necessary if
// you are writing your own client.

DEFINE_string(file, "/tmp/python_rpc",
              "File written to by this binary, read by client.");
// This file signals to `call_python_full_test.sh` that a full execution has
// been completed. This is useful for the `threading-loop` case, where we want
// send a Ctrl+C interrupt only when finished.
DEFINE_string(done_file, "/tmp/python_rpc_done",
              "Signifies last Python command has been executed.");
// Ensure that we test error behavior.
DEFINE_bool(with_error, false, "Inject an error towards the end.");
DEFINE_bool(sleep_at_end, false,
            "Sleep at end to check behavior of C++ if the Python client "
            "fails.");

// TODO(eric.cousineau): Instrument client to verify output (and make this a
// unittest).

namespace drake {
namespace common {

// N.B. This is NOT necessary for normal use; it is only done for testing.
// If you use any `CallPython` calls prior to `CallPythonInit`, then the
// default pipe will be used.
GTEST_TEST(TestCallPython, Start) {
  CallPythonInit(FLAGS_file);
  // Tell client to expect a finishing signal.
  CallPython("execution_check.start");
}

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
  // Print all variables available.
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

  // Place error code toward the end, so that the test fails if this is not
  // processed.
  if (FLAGS_with_error) {
    CallPython("bad_function_name");
  }

  CallPython("print", "Third column should now be [1, 2, 3]: ");
  magic.slice(":", 2) = Eigen::Vector3d(1, 2, 3);
  CallPython("print", magic);

  // Send variables in different ways.
  CallPython("print", "Variable setting:");
  CallPython("setvar", "a1", "abc");
  CallPython("setvars", "a2", "def", "a3", "ghi");
  CallPython("exec", "a4 = 'jkl'");
  CallPython("locals")["a5"] = "mno";
  CallPython("locals").attr("update")(ToPythonKwargs("a6", "pqr"));
  CallPython("eval", "print(a1 + a2 + a3 + a4 + a5 + a6)");

  // Test deleting variables.
  CallPython("exec", "assert 'a6' in locals()");
  CallPython("exec", "del a6");
  CallPython("exec", "assert 'a6' not in locals()");
  CallPython("print", "Deleted variable");

  // Test primitive assignment.
  CallPython("setvar", "b1", 10);
  CallPython("exec", "b1 += 20");
  CallPython("exec", "assert b1 == 30");
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
  CallPython("clf");
  CallPython("plot", time, val);
  // Send variables.
  CallPython("setvars", "time", time, "val", val);
  CallPython("eval", "print(len(time) + len(val))");
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
  CallPython("clf");
  CallPython("surf", x, y, Z);
  // Send variables.
  CallPython("setvars", "x", x, "y", y, "Z", Z);
  CallPython("eval", "print(len(x) + len(y) + len(Z))");
}

// N.B. This is NOT necessary for normal use; it is only done for testing.
GTEST_TEST(TestCallPython, Finish) {
  if (FLAGS_sleep_at_end) {
    // If the Python client closes before this program closes the FIFO pipe,
    // then this executable should receive a SIGPIPE signal (and fail).
    // Sleeping here simulates this condition for manual testing.
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  // Signal finishing to client.
  CallPython("execution_check.finish");
  // Signal finishing to `call_python_full_test.sh`.
  // Use persistence to increment the number of times this has been executed.
  CallPython("setvar", "done_file", FLAGS_done_file);
  CallPython("exec", R"""(
if 'done_count' in locals():
  done_count += 1
else:
  done_count = 1
with open(done_file, 'w') as f:
  f.write(str(done_count))
)""");
}

}  // namespace common
}  // namespace drake
