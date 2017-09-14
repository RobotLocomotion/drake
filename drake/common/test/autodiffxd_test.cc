#include "drake/common/autodiffxd.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace common {
namespace {

class AutoDiffXdTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // We only set the derivatives of y and leave x's uninitialized.
    y_xd_.derivatives() = Eigen::VectorXd::Ones(3);
    y_3d_.derivatives() = Eigen::VectorXd::Ones(3);
  }

  // Evaluates a given function f with values of AutoDiffXd and values with
  // AutoDiffd<3>. It checks if the values and the derivatives of those
  // evaluation results are matched.
  template <typename F>
  ::testing::AssertionResult Check(const F& f) {
    const AutoDiffXd e1{f(x_xd_, y_xd_)};
    const AutoDiffd<3> e2{f(x_3d_, y_3d_)};
    if (e1.value() != e2.value()) {
      return ::testing::AssertionFailure()
             << "Values do not match: " << e1.value() << " and " << e2.value();
    }
    if (e1.derivatives() != e2.derivatives()) {
      return ::testing::AssertionFailure() << "Derivatives do not match:\n"
                                           << e1.derivatives() << "\n----\n"
                                           << e2.derivatives() << "\n";
    }
    return ::testing::AssertionSuccess();
  }

  // AutoDiffXd constants -- x and y.
  const AutoDiffXd x_xd_{0.5};
  AutoDiffXd y_xd_{0.7};
  // AutoDiffd<3> constants -- x and y.
  const AutoDiffd<3> x_3d_{x_xd_.value()};
  AutoDiffd<3> y_3d_{y_xd_.value()};
};

// We need to specify the return type of the polymorphic lambda function that is
// passed to AutoDiffXdTest::Check() method.
#define DRAKE_NO_REF_ADS_TYPE(x) \
  typename Eigen::internal::remove_reference<decltype(x)>::type

TEST_F(AutoDiffXdTest, Addition) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) + (y + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) + (x + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x + 3.0 + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 + x + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x + (y + 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x + (3.0 + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 + (y + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 + (x + y);
      }));
}

TEST_F(AutoDiffXdTest, Subtraction) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) - (y - x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) - (x - y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x - 3.0 - y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 - x - y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x - (y - 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x - (3.0 - y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 - (y - x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 - (x - y);
      }));
}

TEST_F(AutoDiffXdTest, Multiplication) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) * (y * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) * (x * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x * 3.0 * y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 * x * y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x * (y * 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x * (3.0 * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 * (y * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 * (x * y);
      }));
}

TEST_F(AutoDiffXdTest, Division) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) / (y * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) / (x * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x / 3.0 / y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 / x / y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x / (y / 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x / (3.0 / y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 / (y / x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 / (x / y);
      }));
}

TEST_F(AutoDiffXdTest, Pow) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + x, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + 0.1, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + y, x + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + 0.1, x + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + x, 2.0) * (y * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + 0.1, 2.0) * (y * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + y, 2.0) * (x * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + 0.1, 2.0) * (x * x);
      }));
  // Note that Eigen's AutoDiffScalar does not provide an implementation for
  // pow(double, ADS). Therefore, we do not provide tests for that here.
}

TEST_F(AutoDiffXdTest, Atan2) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(x + x, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(x + 0.1, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(y + y, x + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(y + 0.1, x + x);
      }));
}

TEST_F(AutoDiffXdTest, Abs) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(x + 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(y + 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Abs2) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(x - x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(x - 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(y - y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(y - 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Sqrt) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(0.1 * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(0.1 * y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Cos) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(-(x + 0.1)) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(y + 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Sin) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(x - x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(x - 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(y - y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(y - 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Exp) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(x * 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(y * 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Log) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(x / x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(x / 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(y / y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(y / 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Tan) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(0.1 + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(0.1 + y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Asin) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(x * 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(y * 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Acos) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(0.1 - x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(-x) + y;
      }));

  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(0.1 - y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Tanh) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(x / x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(0.1 + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(y - 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Sinh) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(x * 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(y / y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(0.5 - y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Cosh) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(0.1 / x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(0.5 - y) + x;
      }));
}

TEST_F(AutoDiffXdTest, min) {
  // Note that x < y and min(x, y) = x in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(x + x, y + y) + y;
      }));  // 2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(y + y, x + x) + y;
      }));  // 2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(x + 0.3, y + 0.3) + y;
      }));  // x + 0.3 + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(y + 0.4, x + 0.4) + y;
      }));  // x + 0.4 + y

  // Note that -y < -x and min(-x, -y) = -y in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-x - x, -y - y) + x;
      }));  // -2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-y - y, -x - x) + x;
      }));  // -2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-(x + 0.5), -(y + 0.5)) + x;
      }));  // -y - 0.5 + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-(y + 0.6), -(x + 0.6)) + x;
      }));  // -y - 0.6 + x
}

TEST_F(AutoDiffXdTest, max) {
  // Note that x < y and max(x, y) = y in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(x + x, y + y) + x;
      }));  // 2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(y + y, x + x) + x;
      }));  // 2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.3 + x, 0.3 + y) + x;
      }));  // 0.3 + y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.4 + y, 0.4 + x) + x;
      }));  // 0.4 + y + x

  // Note that -y < -x and max(-x, -y) = -x in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(-x - x, -y - y) + y;
      }));  // -2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(-y - y, -x - x) + y;
      }));  // -2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.5 - x, 0.5 - y) + y;
      }));  // 0.5 -x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.6 - y, 0.6 - x) + y;
      }));  // 0.6 -x + y
}

#undef DRAKE_NO_REF_ADS_TYPE

}  // namespace
}  // namespace common
}  // namespace drake
