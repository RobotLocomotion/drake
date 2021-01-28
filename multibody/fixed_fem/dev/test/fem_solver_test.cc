#include "drake/multibody/fixed_fem/dev/fem_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fixed_fem/dev/eigen_conjugate_gradient_solver.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/linear_simplex_element.h"
#include "drake/multibody/fixed_fem/dev/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_model.h"
#include "drake/multibody/fixed_fem/dev/zeroth_order_state_updater.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
using T = AutoDiffXd;
using QuadratureType =
    SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
constexpr int kNumQuads = QuadratureType::num_quadrature_points();
using IsoparametricElementType =
    LinearSimplexElement<T, kNaturalDimension, kSpatialDimension, kNumQuads>;
using ConstitutiveModelType = LinearConstitutiveModel<T, kNumQuads>;
using ElementType =
    StaticElasticityElement<IsoparametricElementType, QuadratureType,
                            ConstitutiveModelType>;
using SolverType = FemSolver<StaticElasticityModel<ElementType>>;
const double kYoungsModulus = 1.23;
const double kPoissonRatio = 0.456;
const T kDensity{1.23};

class FemSolverTest : public ::testing::Test {
 protected:
  /* Make a box and subdivide it into 6 tetrahedra. */
  static geometry::VolumeMesh<T> MakeBoxTetMesh() {
    const double length = 0.1;
    geometry::Box box(length, length, length);
    geometry::VolumeMesh mesh =
        geometry::internal::MakeBoxVolumeMesh<T>(box, length);
    return mesh;
  }

  void SetUp() override {
    /* Builds the FemModel. */
    geometry::VolumeMesh<T> mesh = MakeBoxTetMesh();
    ConstitutiveModelType constitutive_model(kYoungsModulus, kPoissonRatio);
    auto model = std::make_unique<StaticElasticityModel<ElementType>>();
    model->AddStaticElasticityElementsFromTetMesh(mesh, constitutive_model,
                                                  kDensity);

    /* Builds the StateUpdater. */
    auto state_updater =
        std::make_unique<ZerothOrderStateUpdater<FemState<ElementType>>>();

    /* Builds the LinearSystemSolver. */
    auto linear_solver =
        std::make_unique<internal::EigenConjugateGradientSolver<T>>();

    solver_ = std::make_unique<SolverType>(
        std::move(model), std::move(state_updater), std::move(linear_solver));
  }

  /* The solver under test. */
  std::unique_ptr<SolverType> solver_;
};

/* Tests the mesh has been successfully converted to elements. */
TEST_F(FemSolverTest, Smoke) {
  const auto state = solver_->EvalFemState();
  std::cout << state.q() << std::endl;
  EXPECT_TRUE(false);
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
