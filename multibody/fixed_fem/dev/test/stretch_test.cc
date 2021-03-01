#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/fixed_fem/dev/fem_solver.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/linear_simplex_element.h"
#include "drake/multibody/fixed_fem/dev/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kSolutionDimension = 3;
constexpr int kQuadratureOrder = 1;
using T = double;
using QuadratureType =
    SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
constexpr int kNumQuads = QuadratureType::num_quadrature_points();
using IsoparametricElementType =
    LinearSimplexElement<T, kNaturalDimension, kSpatialDimension, kNumQuads>;
using ConstitutiveModelType = LinearConstitutiveModel<T, kNumQuads>;
using ElementType =
    StaticElasticityElement<IsoparametricElementType, QuadratureType,
                            ConstitutiveModelType>;
using ModelType = StaticElasticityModel<ElementType>;
using geometry::VolumeMesh;
const double kYoungsModulus = 7.89;
const double kPoissonRatio = 0.4;
const double kDensity = 5.43;
const double kTol = 1e-14;
/* Side length of the bar. */
const double kL = 1.23;
const double kStretchFactor = 3.45;

class StretchTest : public ::testing::Test {
 protected:
  /* Make a box with side length kL and subdivide it into a tetrahedra mesh. */
  static VolumeMesh<T> MakeBoxTetMesh() {
    geometry::Box box(kL, kL, kL);
    /* It's important to set kL/dx to be a power of 2, so that the appropriate
     boundary conditions at x = 0 and z = 0 can be applied. The test, however,
     does pass without those boundary conditions as well even though the
     solution is not unique. */
    const double dx = kL / 4;
    const VolumeMesh<T> mesh =
        geometry::internal::MakeBoxVolumeMesh<T>(box, dx);
    return mesh;
  }

  /* Create the FEM model of the geometry being stretched. */
  static std::unique_ptr<ModelType> MakeBoxFemModel() {
    const ConstitutiveModelType constitutive_model(kYoungsModulus,
                                                   kPoissonRatio);
    const VolumeMesh<T> mesh = MakeBoxTetMesh();
    auto model = std::make_unique<ModelType>();
    model->AddStaticElasticityElementsFromTetMesh(mesh, constitutive_model,
                                                  kDensity);
    model->SetGravity({0, 0, 0});
    return model;
  }

  /* The rest positions of the vertices of the bar. */
  static Matrix3X<T> MakeRestPositions() {
    const std::unique_ptr<ModelType> model = MakeBoxFemModel();
    const std::unique_ptr<FemStateBase<T>> state = model->MakeFemStateBase();
    return Eigen::Map<const Matrix3X<T>>(state->q().data(), 3,
                                         state->q().size() / 3);
  }

  void SetUp() override {
    /* Set up model. */
    std::unique_ptr<ModelType> model = MakeBoxFemModel();
    const std::unique_ptr<FemStateBase<T>> state = model->MakeFemStateBase();
    model->SetDirichletBoundaryCondition(MakeStretchBc(state->q()));

    /* Set up solver. */
    solver_ = std::make_unique<FemSolver<T>>(std::move(model));
    solver_->set_linear_solve_tolerance(kTol);
    solver_->set_relative_tolerance(kTol);
    solver_->set_absolute_tolerance(kTol);
  }

  /* Given the rest positions, creates a Dirichlet boundary condition that
   stretches the two ends of the bar in the y-direction by a factor of
   kStretchFactor. */
  std::unique_ptr<DirichletBoundaryCondition<T>> MakeStretchBc(
      const VectorX<T>& q) {
    auto bc =
        std::make_unique<DirichletBoundaryCondition<T>>(/* ODE order */ 0);
    const int num_dofs = q.size();
    const double kEpsilon = 1e-14;
    for (int node = 0; node < num_dofs / kSolutionDimension; ++node) {
      const DofIndex x_dof_index(kSolutionDimension * node);
      const DofIndex y_dof_index(kSolutionDimension * node + 1);
      const DofIndex z_dof_index(kSolutionDimension * node + 2);
      /* No shift in the xz-plane. */
      if (std::abs(q(x_dof_index)) <= kEpsilon) {
        bc->AddBoundaryCondition(x_dof_index, Vector1<T>(q(x_dof_index)));
      }
      /* Stretch the two ends of the bar in y-direction. */
      if (std::abs(std::abs(q(y_dof_index)) - std::abs(kL / 2)) <= kEpsilon) {
        bc->AddBoundaryCondition(y_dof_index,
                                 Vector1<T>(kStretchFactor * q(y_dof_index)));
      }
      /* No shift in the xz-plane. */
      if (std::abs(q(z_dof_index)) <= kEpsilon) {
        bc->AddBoundaryCondition(z_dof_index, Vector1<T>(q(z_dof_index)));
      }
    }
    return bc;
  }

  /* The solver under test. */
  std::unique_ptr<FemSolver<T>> solver_;
};

/* Tests that FEM solution matches the analytical solution provided in
 doc/stretch_test.pdf. */
TEST_F(StretchTest, Stretch) {
  const FemModelBase<T>& model = solver_->model();
  std::unique_ptr<FemStateBase<T>> state = model.MakeFemStateBase();
  solver_->SolveStaticModelWithInitialGuess(state.get());

  /* The equilibrium positions and the rest positions. */
  const Matrix3X<T> q = Eigen::Map<const Matrix3X<T>>(state->q().data(), 3,
                                                      state->q().size() / 3);
  const Matrix3X<T> Q = MakeRestPositions();

  /* According to the pen and paper calculation, we should expect u₁ =
   -νx(kStretchFactor-1), u₂ = y(kStretchFactor-1), and u₃ =
   -νz(kStretchFactor-1). */
  EXPECT_TRUE(CompareMatrices(q.row(1), Q.row(1) * kStretchFactor, kL * kTol));
  /* The displacement in x,y,z direction. */
  const VectorX<T> u0 = q.row(0) - Q.row(0);
  const VectorX<T> u1 = q.row(1) - Q.row(1);
  const VectorX<T> u2 = q.row(2) - Q.row(2);
  EXPECT_TRUE(CompareMatrices(u0.transpose(),
                              Q.row(0) * -kPoissonRatio * (kStretchFactor - 1),
                              kL * kTol));
  EXPECT_TRUE(CompareMatrices(u1.transpose(), Q.row(1) * (kStretchFactor - 1),
                              kL * kTol));
  EXPECT_TRUE(CompareMatrices(u2.transpose(),
                              Q.row(2) * -kPoissonRatio * (kStretchFactor - 1),
                              kL * kTol));
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
