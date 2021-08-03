#include "drake/multibody/fixed_fem/dev/fem_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/eigen_conjugate_gradient_solver.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_model.h"
#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kSolutionDimension = 3;
constexpr int kQuadratureOrder = 1;
/* Number of nodes subject to Dirichlet BC. */
constexpr int kNumDirichlet = 4;
constexpr int kNumNodes = 8;
constexpr int kNumDofs = kNumNodes * kSolutionDimension;
constexpr double kTol = 1e-14;
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
using State = FemState<ElementType>;
const double kYoungsModulus = 1.234;
const double kPoissonRatio = 0.4567;
const double kDensity = 8.90;
/* Set a non-default gravity to check gravity's set correctly. */
const Vector3<T> kGravity{1.23, -4.56, 7.89};

class FemSolverTest : public ::testing::Test {
 protected:
  /* Make a box and subdivide it into 6 tetrahedra. */
  static geometry::VolumeMesh<T> MakeBoxTetMesh() {
    const double kLength = 0.1;
    const geometry::Box box(kLength, kLength, kLength);
    const geometry::VolumeMesh<T> mesh =
        geometry::internal::MakeBoxVolumeMesh<T>(box, kLength);
    EXPECT_EQ(mesh.num_elements(), 6);
    EXPECT_EQ(mesh.num_vertices(), 8);
    return mesh;
  }

  void SetUp() override {
    /* Builds the FemModel. */
    const geometry::VolumeMesh<T> mesh = MakeBoxTetMesh();
    const ConstitutiveModelType constitutive_model(kYoungsModulus,
                                                   kPoissonRatio);
    model_.AddStaticElasticityElementsFromTetMesh(mesh, constitutive_model,
                                                  kDensity);
    model_.SetGravity(kGravity);
    /* Set up the Dirichlet BC. */
    std::unique_ptr<DirichletBoundaryCondition<T>> bc = MakeCeilingBc();
    model_.SetDirichletBoundaryCondition(std::move(bc));

    /* Builds the FemSolver. */
    solver_.set_linear_solve_tolerance(kTol);
    solver_.set_relative_tolerance(kTol);
    solver_.set_absolute_tolerance(kTol);
  }

  /* Creates the undeformed state of the model under test. */
  State MakeReferenceState() const { return model_.MakeFemState(); }

  /* Creates a Dirichlet boundary condition that constrains the first
   `kNumDirichlet` vertices. */
  std::unique_ptr<DirichletBoundaryCondition<T>> MakeCeilingBc() const {
    auto bc = std::make_unique<DirichletBoundaryCondition<T>>(0);
    const State state = MakeReferenceState();
    const VectorX<T>& q = state.q();
    for (int node = 0; node < kNumDirichlet; ++node) {
      const DofIndex starting_dof_index(kSolutionDimension * node);
      /* Dirichlet BC for all dofs associated with the node. */
      for (int d = 0; d < kSolutionDimension; ++d) {
        bc->AddBoundaryCondition(DofIndex(starting_dof_index + d),
                                 Vector1<T>(q(starting_dof_index + d)));
      }
    }
    return bc;
  }

  /* Creates an arbitrary state that respects the Dirichlet BC created above. */
  State MakeArbitraryState() const {
    State state = MakeReferenceState();
    const VectorX<T> q = MakeArbitraryPositions();
    state.SetQ(q);
    std::unique_ptr<DirichletBoundaryCondition<T>> bc = MakeCeilingBc();
    state.ApplyBoundaryCondition(*bc);
    return state;
  }

  /* Creates an arbitrary position vector with dofs compatible with the box
   mesh. The positions for this test are indeed arbitrarily chosen and do not
   necessarily lead to a valid physical configuration. Whether the configuration
   is physical is not relevant because the static linear elasticity test below
   should pass *regardless* of the geometry state. */
  static Vector<double, kNumDofs> MakeArbitraryPositions() {
    Vector<double, kNumDofs> q;
    q << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67,
        0.81, 0.36, 0.45, 0.31, 0.29, 0.71, 0.30, 0.68, 0.58, 0.52, 0.35, 0.76;
    return q;
  }

  ModelType model_;
  /* The solver under test. */
  FemSolver<T> solver_{&model_};
};

namespace {
/* We move the vertices of the mesh to arbitrary locations q and record the net
force f exerted on the vertices. Then if we apply f on the vertices in the
reference state, we should recover the positions q for static equilibrium. */
TEST_F(FemSolverTest, StaticForceEquilibrium) {
  /* Create an arbitrary state and find the nodel force exerted on the vertices
   of the mesh (in unit N). */
  const State prescribed_state = MakeArbitraryState();
  VectorX<T> nodal_force(kNumDofs);
  model_.CalcResidual(prescribed_state, &nodal_force);

  /* If we exert the same force on the reference state, we should expect to
   recover the same positions as above. */
  const T initial_error = nodal_force.norm();
  model_.SetExplicitExternalForce(nodal_force);
  State state = MakeReferenceState();
  solver_.SolveStaticModelWithInitialGuess(&state);
  EXPECT_TRUE(CompareMatrices(state.q(), prescribed_state.q(),
                              std::max(kTol, kTol * initial_error)));
}

/* Verifies that methods that take FemStateBase as argument throw if the
 concrete state type is incompatible with the model type. */
TEST_F(FemSolverTest, IncompatibleState) {
  FemState<test::DummyElement<0>> dummy_state(Vector3<double>(1, 2, 3));
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.SolveStaticModelWithInitialGuess(&dummy_state),
      "SolveStaticModelWithInitialGuess\\(\\): The type of the FemState is "
      "incompatible "
      "with the type of the FemModel.");
  State state_with_wrong_size(Vector3<double>(1, 2, 3));
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.SolveStaticModelWithInitialGuess(&state_with_wrong_size),
      "SolveStaticModelWithInitialGuess\\(\\): The size of the "
      "FemState \\(3\\) is incompatible "
      "with the size of the FemModel \\(24\\).");
}
// TODO(xuchenhan-tri): Add unit test for AdvanceOneTimeStep().
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
