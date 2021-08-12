#include "drake/multibody/fixed_fem/dev/deformable_model.h"

#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/corotated_model.h"
#include "drake/multibody/fixed_fem/dev/dirichlet_boundary_condition.h"
#include "drake/multibody/fixed_fem/dev/dynamic_elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/dynamic_elasticity_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
DeformableBodyIndex DeformableModel<T>::RegisterDeformableBody(
    internal::ReferenceDeformableGeometry<T> geometry, std::string name,
    const DeformableBodyConfig<T>& config,
    geometry::ProximityProperties proximity_props) {
  /* Throw if name is not unique. */
  for (int i = 0; i < num_bodies(); ++i) {
    if (name == names_[i]) {
      throw std::runtime_error(fmt::format(
          "{}(): A body with name '{}' already exists in the system.", __func__,
          name));
    }
  }
  DeformableBodyIndex body_index(num_bodies());
  switch (config.material_model()) {
    case MaterialModel::kLinear:
      RegisterDeformableBodyHelper<internal::LinearConstitutiveModel>(
          geometry.mesh(), std::move(name), config);
      break;
    case MaterialModel::kCorotated:
      RegisterDeformableBodyHelper<internal::CorotatedModel>(
          geometry.mesh(), std::move(name), config);
      break;
  }
  proximity_properties_.emplace_back(std::move(proximity_props));
  reference_configuration_geometries_.emplace_back(std::move(geometry));
  return body_index;
}

template <typename T>
void DeformableModel<T>::SetWallBoundaryCondition(DeformableBodyIndex body_id,
                                                  const Vector3<T>& p_WQ,
                                                  const Vector3<T>& n_W,
                                                  double distance_tolerance) {
  DRAKE_DEMAND(n_W.norm() > 1e-10);
  const Vector3<T>& n_hatW = n_W.normalized();
  DRAKE_THROW_UNLESS(body_id < num_bodies());
  const int kDim = 3;
  auto& fem_model = fem_models_[body_id];
  const int num_nodes = fem_model->num_nodes();
  // TODO(xuchenhan-tri): FemModel should support an easier way to retrieve its
  //  reference positions.
  const std::unique_ptr<FemStateBase<T>> fem_state =
      fem_model->MakeFemStateBase();
  const VectorX<T>& initial_positions = fem_state->q();
  auto bc = std::make_unique<DirichletBoundaryCondition<T>>(/* ODE order */ 2);
  for (int n = 0; n < num_nodes; ++n) {
    const Vector3<T>& p_WV = initial_positions.template segment<kDim>(n * kDim);
    const T distance_to_wall = (p_WV - p_WQ).dot(n_hatW);
    if (distance_to_wall * distance_to_wall <
        distance_tolerance * distance_tolerance) {
      const int dof_index(kDim * n);
      for (int d = 0; d < kDim; ++d) {
        bc->AddBoundaryCondition(DofIndex(dof_index + d),
                                 Vector3<T>(p_WV(d), 0, 0));
      }
    }
  }
  fem_model->SetDirichletBoundaryCondition(std::move(bc));
}

template <typename T>
int DeformableModel<T>::NumDofs() const {
  int dofs = 0;
  for (const auto& fem_model : fem_models_) {
    dofs += fem_model->num_dofs();
  }
  return dofs;
}

template <typename T>
template <template <class, int> class Model>
void DeformableModel<T>::RegisterDeformableBodyHelper(
    const geometry::VolumeMesh<T>& mesh, std::string name,
    const DeformableBodyConfig<T>& config) {
  constexpr int kNaturalDimension = 3;
  constexpr int kSpatialDimension = 3;
  constexpr int kQuadratureOrder = 1;
  using QuadratureType =
      internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
  constexpr int kNumQuads = QuadratureType::num_quadrature_points;
  using IsoparametricElementType =
      internal::LinearSimplexElement<T, kNaturalDimension, kSpatialDimension,
                                     kNumQuads>;
  using ConstitutiveModelType = Model<T, kNumQuads>;
  static_assert(
      std::is_base_of_v<
          internal::ConstitutiveModel<ConstitutiveModelType,
                                      typename ConstitutiveModelType::Traits>,
          ConstitutiveModelType>,
      "The template parameter 'Model' must be derived from "
      "ConstitutiveModel.");
  using ElementType =
      DynamicElasticityElement<IsoparametricElementType, QuadratureType,
                               ConstitutiveModelType>;
  using FemModelType = DynamicElasticityModel<ElementType>;
  using StateType = FemState<ElementType>;

  const DampingModel<T> damping_model(config.mass_damping_coefficient(),
                                      config.stiffness_damping_coefficient());
  auto fem_model = std::make_unique<FemModelType>(plant_->time_step());
  // TODO(xuchenhan-tri): Any changes to the gravity will not reflect on the
  //  deformable bodies added before the gravity change. This needs to be fixed.
  fem_model->SetGravity(plant_->gravity_field().gravity_vector());
  ConstitutiveModelType constitutive_model(config.youngs_modulus(),
                                           config.poisson_ratio());
  fem_model->AddDynamicElasticityElementsFromTetMesh(
      mesh, constitutive_model, config.mass_density(), damping_model);

  const StateType state = fem_model->MakeFemState();
  const int num_dofs = state.num_generalized_positions();
  VectorX<T> discrete_state(num_dofs * 3);
  discrete_state.head(num_dofs) = state.q();
  discrete_state.segment(num_dofs, num_dofs) = state.qdot();
  discrete_state.tail(num_dofs) = state.qddot();
  model_discrete_states_.emplace_back(discrete_state);

  fem_models_.emplace_back(std::move(fem_model));
  names_.emplace_back(std::move(name));
}

template <typename T>
void DeformableModel<T>::DoDeclareSystemResources(MultibodyPlant<T>* plant) {
  /* Ensure that the owning plant is the one declaring system resources. */
  DRAKE_DEMAND(plant == plant_);
  /* Declare output ports. */
  vertex_positions_port_ = &this->DeclareAbstractOutputPort(
      plant, "vertex_positions",
      []() { return AbstractValue::Make<std::vector<VectorX<T>>>(); },
      [this](const systems::Context<T>& context, AbstractValue* output) {
        std::vector<VectorX<T>>& output_value =
            output->get_mutable_value<std::vector<VectorX<T>>>();
        output_value.resize(num_bodies());
        const systems::DiscreteValues<T>& all_discrete_states =
            context.get_discrete_state();
        for (int i = 0; i < num_bodies(); ++i) {
          const auto& discrete_value =
              all_discrete_states.get_value(discrete_state_indexes_[i]);
          const int num_dofs = discrete_value.size() / 3;
          const auto& q = discrete_value.head(num_dofs);
          output_value[i] = q;
        }
      },
      {systems::System<double>::xd_ticket()});

  /* Declare discrete states. */
  for (int i = 0; i < num_bodies(); ++i) {
    discrete_state_indexes_.emplace_back(
        this->DeclareDiscreteState(plant, model_discrete_states_[i]));
  }
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::DeformableModel);
