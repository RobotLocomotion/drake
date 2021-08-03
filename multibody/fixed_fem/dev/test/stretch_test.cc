#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/fem_solver.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kSolutionDimension = 3;
constexpr int kQuadratureOrder = 1;
using T = double;
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
constexpr int kNumQuads = QuadratureType::num_quadrature_points;
using IsoparametricElementType =
    internal::LinearSimplexElement<T, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using ConstitutiveModelType = internal::LinearConstitutiveModel<T, kNumQuads>;
using ElementType =
    StaticElasticityElement<IsoparametricElementType, QuadratureType,
                            ConstitutiveModelType>;
using ModelType = StaticElasticityModel<ElementType>;
using geometry::VolumeMesh;
const double kYoungsModulus = 1e8;  // Unit Pa.
const double kPoissonRatio = 0.4;
const double kDensity = 1e3;  // Unit kg.
const double kTol = 1e-14;
/* Side length of the bar with unit m. */
const double kLx = 1;
const double kLy = 4;
const double kLz = 2;
/* Resolution hint for meshing the box. */
const double kDx = 0.5;

const double kStretchFactor = 3.45;
const double kEpsilon = kStretchFactor - 1;

class StretchTest : public ::testing::Test {
 protected:
  /* Make a quarter of the box with size kLx * kLy * kLz as in the accompanying
   analytical solution document in doc/stretch_bar_test.tex and subdivide it
   into a tetrahedral mesh. */
  static VolumeMesh<T> MakeBoxTetMesh() {
    geometry::Box box(kLx / 2.0, kLy, kLz / 2.0);
    const internal::ReferenceDeformableGeometry<T> geometry =
        MakeDiamondCubicBoxDeformableGeometry(
            box, kDx,
            math::RigidTransform(Vector3<double>(kLx / 4.0, 0, kLz / 4.0)));
    return geometry.mesh();
  }

  /* The rest positions of the vertices of the bar. */
  Matrix3X<T> MakeRestPositions() const {
    const std::unique_ptr<FemStateBase<T>> state = model_.MakeFemStateBase();
    return Eigen::Map<const Matrix3X<T>>(state->q().data(), 3,
                                         state->q().size() / 3);
  }

  void SetUp() override {
    /* Set up model. */
    const ConstitutiveModelType constitutive_model(kYoungsModulus,
                                                   kPoissonRatio);
    const VolumeMesh<T> mesh = MakeBoxTetMesh();
    model_.AddStaticElasticityElementsFromTetMesh(mesh, constitutive_model,
                                                  kDensity);
    model_.SetGravity({0, 0, 0});
    const std::unique_ptr<FemStateBase<T>> state = model_.MakeFemStateBase();
    model_.SetDirichletBoundaryCondition(MakeStretchBc(state->q()));

    /* Set up solver. */
    solver_.set_linear_solve_tolerance(kTol);
    solver_.set_relative_tolerance(kTol);
    solver_.set_absolute_tolerance(kTol);
  }

  /* Given the rest positions, creates a Dirichlet boundary condition that
   stretches the two ends of the bar in the y-direction by a factor of
   kStretchFactor. */
  std::unique_ptr<DirichletBoundaryCondition<T>> MakeStretchBc(
      const VectorX<T>& q) {
    auto bc =
        std::make_unique<DirichletBoundaryCondition<T>>(/* ODE order */ 0);
    const int num_dofs = q.size();
    for (int node = 0; node < num_dofs / kSolutionDimension; ++node) {
      const DofIndex x_dof_index(kSolutionDimension * node);
      const DofIndex y_dof_index(kSolutionDimension * node + 1);
      const DofIndex z_dof_index(kSolutionDimension * node + 2);
      /* No translation in the xz-plane. */
      if (std::abs(q(int{x_dof_index})) <= kTol) {
        bc->AddBoundaryCondition(x_dof_index, Vector1<T>(q(int{x_dof_index})));
      }
      /* Stretch the two ends of the bar in y-direction. */
      if (std::abs(std::abs(q(int{y_dof_index})) - std::abs(kLy / 2)) <= kTol) {
        bc->AddBoundaryCondition(
            y_dof_index, Vector1<T>(kStretchFactor * q(int{y_dof_index})));
      }
      /* No translation in the xz-plane. */
      if (std::abs(q(int{z_dof_index})) <= kTol) {
        bc->AddBoundaryCondition(z_dof_index, Vector1<T>(q(int{z_dof_index})));
      }
    }
    return bc;
  }

  ModelType model_;
  /* The solver under test. */
  FemSolver<T> solver_{&model_};
};

/* Tests that FEM solution matches the analytical solution provided in
 doc/stretch_bar_test.pdf. */
TEST_F(StretchTest, Stretch) {
  std::unique_ptr<FemStateBase<T>> state = model_.MakeFemStateBase();
  solver_.SolveStaticModelWithInitialGuess(state.get());

  /* The equilibrium positions and the rest positions. */
  const Matrix3X<T> q = Eigen::Map<const Matrix3X<T>>(state->q().data(), 3,
                                                      state->q().size() / 3);
  const Matrix3X<T> Q = MakeRestPositions();

  /* According to the pen and paper calculation, we should expect u₁ =
   -νx * kEpsilon, u₂ = y * kEpsilon, and u₃ = -νz * kEpsilon. */
  /* The displacement in x,y,z direction. */
  const VectorX<T> u0 = q.row(0) - Q.row(0);
  const VectorX<T> u1 = q.row(1) - Q.row(1);
  const VectorX<T> u2 = q.row(2) - Q.row(2);
  EXPECT_TRUE(CompareMatrices(
      u0.transpose(), Q.row(0) * -kPoissonRatio * kEpsilon, kLx * kTol));
  EXPECT_TRUE(CompareMatrices(u1.transpose(), Q.row(1) * kEpsilon, kLy * kTol));
  EXPECT_TRUE(CompareMatrices(
      u2.transpose(), Q.row(2) * -kPoissonRatio * kEpsilon, kLz * kTol));
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
