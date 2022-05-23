#include "drake/multibody/fem/mpm-dev/CorotatedModel.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double TOLERANCE = 1e-12;

GTEST_TEST(CorotatedModelTest, CorotatedModelCalculationTest) {
    // Taken from fem/test/corotated_model_data_test.cc
    /* We set deformation gradient as F = R*S where R is an arbitrary rotation
    matrix and S is an arbitrary sysmmetric positive definite matrix. */
    const Matrix3<double> R =
        math::RotationMatrix<double>
            (math::RollPitchYaw<double>(pi/2.0, pi, 3.0*pi/2.0)).matrix();
    const Matrix3<double> S = (Matrix3<double>() <<
            6, 1, 2,
            1, 4, 1,
            2, 1, 5).finished();
    const Matrix3<double> P_exact1 = (Matrix3<double>() <<
            -4, -2, -8,
            10,  2,  4,
            -2, -6, -2).finished();
    const Matrix3<double> P_exact2 = (Matrix3<double>() <<
            1336,   764, -4432,
            3668,  -572, -1336,
            572,  -5004,   764).finished();
    const Matrix3<double> tau_exact1 = (Matrix3<double>() <<
             50, -42,  20,
            -42,  70, -22,
             20, -22,  28).finished();
    const Matrix3<double> tau_exact2 = (Matrix3<double>() <<
             18724, -84,  40,
            -84,  18764, -44,
             40, -44,  18680).finished();
    const Matrix3<double> F = R * S;
    Matrix3<double> F2;
    Matrix3<double> P, tau;

    // First test on E = 2.0, nu = 0.0
    CorotatedModel corotated_model = CorotatedModel(2.0, 0.0);
    corotated_model.CalcFirstPiolaKirchhoffStress(F, &P);
    corotated_model.CalcKirchhoffStress(F, &tau);
    EXPECT_TRUE(CompareMatrices(P, P_exact1, TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau_exact1, TOLERANCE));

    // Next test on E = 5.0, nu = 0.25
    corotated_model = CorotatedModel(5.0, 0.25);
    corotated_model.CalcFirstPiolaKirchhoffStress(F, &P);
    corotated_model.CalcKirchhoffStress(F, &tau);
    EXPECT_TRUE(CompareMatrices(P, P_exact2, TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau_exact2, TOLERANCE));

    // Sanity check: If F is a rotation matrix, then stresses are zero
    corotated_model = CorotatedModel(5.0, 0.25);
    F2 = math::RotationMatrix<double>
                (math::RollPitchYaw<double>(1.0, 2.0, 3.0)).matrix();
    corotated_model.CalcFirstPiolaKirchhoffStress(F2, &P);
    corotated_model.CalcKirchhoffStress(F2, &tau);
    EXPECT_TRUE(CompareMatrices(P, Matrix3<double>::Zero(), TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, Matrix3<double>::Zero(), TOLERANCE));

    corotated_model = CorotatedModel(23.4, 0.41);
    F2 = math::RotationMatrix<double>
                (math::RollPitchYaw<double>(0.1, -2.4, 13.3)).matrix();
    corotated_model.CalcFirstPiolaKirchhoffStress(F2, &P);
    corotated_model.CalcKirchhoffStress(F2, &tau);
    EXPECT_TRUE(CompareMatrices(P, Matrix3<double>::Zero(), TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, Matrix3<double>::Zero(), TOLERANCE));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
