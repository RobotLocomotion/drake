#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fixed_fem/dev/deformable_body_config.h"
#include "drake/multibody/fixed_fem/dev/fem_model_base.h"
#include "drake/multibody/plant/physical_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %DeformableModel implements the interface in PhysicalModel and provides the
 functionalities to specify deformable bodies. Each deformable body is modeled
 as a volumetric mesh.

 Deformable bodies can only be added, but not deleted. Each deformable body is
 uniquely identified by its index, which is equal to the number of deformable
 bodies existing in the %DeformableModel at the time of its registration.

 The current positions of the vertices of the mesh representing the deformable
 bodies can be queried via the `vertex_positions` output port. The output port
 is an abstract-valued port containing std::vector<VectorX<T>>. There is one
 VectorX for each deformable body registered with the system. The i-th body
 corresponds to the i-th VectorX. The i-th VectorX has 3N values where N is the
 number of vertices in the i-th mesh. For mesh i, the x-, y-, and z-positions
 (measured and expressed in the world frame) of the j-th vertex are 3j, 3j + 1,
 and 3j + 2 in the i-th VectorX from the output port.

 The connectivity of the meshes representing the deformable bodies and their
 initial positions can be queried via `initial_meshes()` which returns an
 std::vector of volume meshes. The i-th mesh stores the connectivity for body i,
 which does not change throughout the simulation, as well as the initial
 positions of the vertices of the i-th body.

 Simple zero DirichletBoundaryCondition can be configured via
 `SetRegisteredBodyInWall()`.

 @tparam_nonsymbolic_scalar. */
template <typename T>
class DeformableModel final : public multibody::internal::PhysicalModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableModel)

  /* Construct a %DeformableModel to be owned by the given MultibodyPlant.
   @pre plant != nullptr.
   @pre plant is discrete. */
  explicit DeformableModel(const MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
    DRAKE_DEMAND(plant_->is_discrete());
  }

  // TODO(xuchenhan-tri): Identify deformable bodies with actual identifiers,
  //  which would make deleting deformable bodies easier to track in the future.
  /** Adds a deformable body modeled with linear simplex element, linear
   quadrature rule and the physical properties specified by the given `config`.
   Returns the index of the newly added deformable body.
   @pre `name` is distinct from names of all previously registered bodies.
   @pre config.IsValid() == true. */
  SoftBodyIndex RegisterDeformableBody(const geometry::VolumeMesh<T>& mesh,
                                       std::string name,
                                       const DeformableBodyConfig<T>& config);

  /** Sets zero Dirichlet boundary conditions for a given body. All vertices in
   the mesh of corresponding to the deformable body with `body_id` within
   `distance_tolerance` (measured in meters) from the halfspace defined by point
   `p_WQ` and outward normal `n_W` will be set with a wall boundary condition.
   @pre n_W.norm() > 1e-10.
   @throw std::exception if body_id >= num_bodies(). */
  void SetWallBoundaryCondition(SoftBodyIndex body_id, const Vector3<T>& p_WQ,
                                const Vector3<T>& n_W,
                                double distance_tolerance = 1e-6);

  /** Returns the number of deformable bodies registered. */
  int num_bodies() const { return initial_meshes_.size(); }

  /** Returns the FEM model of the selected deformable body. */
  const FemModelBase<T>& fem_model(SoftBodyIndex body_index) const {
    DRAKE_DEMAND(body_index < num_bodies());
    return *fem_models_[body_index];
  }

  /** Returns the discrete state indexes of all the deformable bodies. */
  std::vector<systems::DiscreteStateIndex> discrete_state_indexes() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return discrete_state_indexes_;
  }

  /** Returns the volume meshes of the deformable bodies at the time of
   registration. The meshes have the same order as the registration of their
   corresponding deformable bodies. */
  const std::vector<geometry::VolumeMesh<T>>& initial_meshes() const {
    return initial_meshes_;
  }

  /** Returns the names of all the registered deformable bodies in the same
   order as the bodies were registered. */
  const std::vector<std::string>& names() const { return names_; }

  /** Returns the `vertex_positions` output port that reports the current
   positions of the vertices of the mesh representing the deformable bodies.
   This output port is an abstract-valued port containing
   std::vector<VectorX<T>>. There is one VectorX for each deformable body
   registered with the system. The i-th body corresponds to the i-th VectorX.
   The i-th VectorX has 3N values where N is the number of vertices in the i-th
   mesh. For mesh i, the x-, y-, and z-positions (measured and expressed in the
   world frame) of the j-th vertex are 3j, 3j + 1, and 3j + 2 in the i-th
   VectorX from the output port. */
  const systems::OutputPort<T>& get_vertex_positions_output_port() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return *vertex_positions_port_;
  }

 private:
  /* Registers a deformable body with the given type of constitutive model.
   @tparam Model  The type of constitutive model for the new deformable body,
                  must be derived from ConstitutiveModel. */
  template <template <class, int> class Model>
  void RegisterDeformableBodyHelper(const geometry::VolumeMesh<T>& mesh,
                                    std::string name,
                                    const DeformableBodyConfig<T>& config);

  void DoDeclareSystemResources(MultibodyPlant<T>* plant) final;

  /* The MultibodyPlant that owns `this` DeformableModel. */
  const MultibodyPlant<T>* plant_{nullptr};
  /* The FemModels owned by this DeformableModel. */
  std::vector<std::unique_ptr<FemModelBase<T>>> fem_models_{};
  /* Initial meshes for all deformable bodies at time of registration. */
  std::vector<geometry::VolumeMesh<T>> initial_meshes_{};
  /* Names of all registered deformable bodies. */
  std::vector<std::string> names_{};
  /* OutputPorts. */
  const systems::OutputPort<T>* vertex_positions_port_;
  /* The discrete state indexes for all deformable bodies. */
  std::vector<systems::DiscreteStateIndex> discrete_state_indexes_;
  /* The state of each deformable body at initialization. */
  std::vector<VectorX<T>> model_discrete_states_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableModel);
