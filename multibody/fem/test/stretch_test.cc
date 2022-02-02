#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"
#include "drake/multibody/fem/fem_solver.h"
#include "drake/multibody/fem/linear_constitutive_model.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/mesh_utilities.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fem/volumetric_element.h"
#include "drake/multibody/fem/volumetric_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
using T = double;
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
constexpr int kNumQuads = QuadratureType::num_quadrature_points;
using IsoparametricElementType =
    internal::LinearSimplexElement<T, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using ConstitutiveModelType = internal::LinearConstitutiveModel<T, kNumQuads>;
using ElementType = VolumetricElement<IsoparametricElementType, QuadratureType,
                                      ConstitutiveModelType>;
using ModelType = VolumetricModel<ElementType>;
using geometry::VolumeMesh;

const double kYoungsModulus = 1e7;  // Unit Pa.
const double kPoissonRatio = 0.45;
const double kDensity = 1e3;  // Unit kg/m^3.
const double kTol = 1e-14;
/* Side length of the bar with unit m. */
const double kLx = 1;
const double kLy = 4;
const double kLz = 2;
/* Resolution hint for meshing the box. */
const double kDx = 0.5;

const double kStretchFactor = 3.45;
const double kEpsilon = kStretchFactor - 1;

/* Parameters for the Newmark-beta integration scheme. */
constexpr double kDt = 0.01;
constexpr double kGamma = 0.5;
constexpr double kBeta = 0.25;

class StretchTest : public ::testing::Test {
 protected:
  /* Make a box with size kLx * kLy * kLz as in the accompanying analytical
   solution document in doc/stretch_bar_test.tex and subdivide it into a
   tetrahedral mesh. */
  static VolumeMesh<T> MakeBoxTetMesh() {
    geometry::Box box(kLx, kLy, kLz);
    return MakeDiamondCubicBoxVolumeMesh<T>(box, kDx);
  }

  void SetUp() override {
    /* Set up model. */
    const ConstitutiveModelType constitutive_model(kYoungsModulus,
                                                   kPoissonRatio);
    const DampingModel<T> damping_model(0.01, 0.02);
    const VolumeMesh<T> mesh = MakeBoxTetMesh();
    model_.AddVolumetricElementsFromTetMesh(mesh, constitutive_model, kDensity,
                                            damping_model);
    model_.SetGravityVector({0, 0, 0});
    const std::unique_ptr<FemState<T>> state = model_.MakeFemState();
    model_.SetDirichletBoundaryCondition(MakeStretchBc(state->GetPositions()));
  }

  /* Given the rest positions, creates a Dirichlet boundary condition that
   stretches the two ends of the bar in the y-direction by a factor of
   kStretchFactor. */
  DirichletBoundaryCondition<T> MakeStretchBc(const VectorX<T>& q) {
    DirichletBoundaryCondition<T> bc;
    const int num_dofs = q.size();
    for (int node = 0; node < num_dofs / kSpatialDimension; ++node) {
      const int x_dof_index = kSpatialDimension * node;
      const int y_dof_index = kSpatialDimension * node + 1;
      const int z_dof_index = kSpatialDimension * node + 2;
      /* No translation in the x=0 plane. */
      if (std::abs(q(x_dof_index)) <= kTol) {
        bc.AddBoundaryCondition(x_dof_index,
                                Vector3<T>(q(x_dof_index), 0.0, 0.0));
      }
      /* Stretch the two ends of the bar in y-direction. */
      if (std::abs(std::abs(q(y_dof_index)) - std::abs(kLy / 2)) <= kTol) {
        bc.AddBoundaryCondition(
            y_dof_index, Vector3<T>(kStretchFactor * q(y_dof_index), 0.0, 0.0));
      }
      /* No translation in the z=0 plane. */
      if (std::abs(q(z_dof_index)) <= kTol) {
        bc.AddBoundaryCondition(z_dof_index,
                                Vector3<T>(q(z_dof_index), 0.0, 0.0));
      }
    }
    return bc;
  }

  ModelType model_;
  AccelerationNewmarkScheme<T> integrator_{kDt, kGamma, kBeta};
  /* The solver under test. */
  FemSolver<T> solver_{&model_, &integrator_};
};

/* Tests that FEM solution matches the analytical solution provided in
 doc/stretch_bar_test.pdf. */
TEST_F(StretchTest, Stretch) {
  std::unique_ptr<FemState<T>> state0 = model_.MakeFemState();
  std::unique_ptr<FemState<T>> state = model_.MakeFemState();
  const auto initial_positions = Eigen::Map<const Matrix3X<T>>(
      state0->GetPositions().data(), 3, model_.num_nodes());
  VectorX<T> expected_positions(state->num_dofs());
  auto expected_positions_matrix =
      Eigen::Map<Matrix3X<T>>(expected_positions.data(), 3, model_.num_nodes());
  /* According to the pen and paper calculation, we should expect u₁ =
   -νx * kEpsilon, u₂ = y * kEpsilon, and u₃ = -νz * kEpsilon. */
  /* The displacement in x,y,z direction. */
  expected_positions_matrix.row(0) =
      initial_positions.row(0) * (1.0 - kPoissonRatio * kEpsilon);
  expected_positions_matrix.row(1) =
      initial_positions.row(1) * (1.0 + kEpsilon);
  expected_positions_matrix.row(2) =
      initial_positions.row(2) * (1.0 - kPoissonRatio * kEpsilon);
  const VectorX<T> expected_velocities = VectorX<T>::Zero(state->num_dofs());
  const VectorX<T> expected_accelerations = VectorX<T>::Zero(state->num_dofs());
  state0->SetPositions(expected_positions);
  state0->SetVelocities(expected_velocities);
  state0->SetAccelerations(expected_accelerations);

  /* Verify that state0 is already the equilibrium state. */
  for (int i = 0; i < 100; ++i) {
    solver_.AdvanceOneTimeStep(*state0, state.get());
    *state0 = *state;
  }
  EXPECT_TRUE(CompareMatrices(state->GetPositions(), expected_positions, kTol));
  EXPECT_TRUE(
      CompareMatrices(state->GetVelocities(), expected_velocities, kTol / kDt));
  EXPECT_TRUE(CompareMatrices(state->GetAccelerations(), expected_accelerations,
                              kTol / (kDt * kDt)));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
