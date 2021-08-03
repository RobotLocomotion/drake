#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fixed_fem/dev/deformable_body_config.h"
#include "drake/multibody/fixed_fem/dev/fem_model_base.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/plant/physical_model.h"

namespace drake {
namespace multibody {
namespace fem {
// TODO(xuchenhan-tri): Consider changing the name to DeformableSolidModel.
/** %DeformableModel implements the interface in PhysicalModel and provides the
 functionalities to specify deformable bodies. Unlike rigid bodies, the shape of
 deformable bodies can change in a simulation. Each deformable body is modeled
 as a volumetric mesh with persisting topology, changing vertex positions, and
 an approximated signed distance field.

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
 initial positions can be queried via `reference_configuration_geometries()`
 which returns the geometries of the deformable bodies at the reference
 configuration. The i-th mesh stores the connectivity for body i, which does not
 change throughout the simulation, as well as the initial positions of the
 vertices of the i-th body.

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
   quadrature rule, the physical properties specified by the given `config`,
   and proximity properties specified by the given `proximity_props`. Returns
   the index of the newly added deformable body.
   @pre config.IsValid() == true.
   @pre `proximity_props` includes the (`material`, `coulomb_friction`) property
   of type CoulombFriction<double>.
   @throw std::exception if `name` is not distinct from names of all previously
   registered bodies. */
  DeformableBodyIndex RegisterDeformableBody(
      internal::ReferenceDeformableGeometry<T> geometry, std::string name,
      const DeformableBodyConfig<T>& config,
      geometry::ProximityProperties proximity_props);

  /** Sets zero Dirichlet boundary conditions for a given body. All vertices in
   the mesh of corresponding to the deformable body with `body_id` within
   `distance_tolerance` (measured in meters) from the halfspace defined by point
   `p_WQ` and outward normal `n_W` will be set with a wall boundary condition.
   @pre n_W.norm() > 1e-10.
   @throw std::exception if body_id >= num_bodies(). */
  void SetWallBoundaryCondition(DeformableBodyIndex body_id,
                                const Vector3<T>& p_WQ, const Vector3<T>& n_W,
                                double distance_tolerance = 1e-6);

  /** Returns the number of deformable bodies registered. */
  int num_bodies() const { return reference_configuration_geometries_.size(); }

  // TODO(xuchenhan-tri): Implement the O(1) `num_dofs()` that precomputes the
  //  result at initialization phase.
  /** Returns the number deformable degrees of freedom. */
  int NumDofs() const;

  /** Returns the FEM model of the selected deformable body. Each deformable
   body is modeled as an individual FEM model. */
  const FemModelBase<T>& fem_model(DeformableBodyIndex body_index) const {
    DRAKE_DEMAND(body_index < num_bodies());
    return *fem_models_[body_index];
  }

  /** Returns the discrete state indexes of all the deformable bodies. There is
   one discrete state per deformable body. */
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes()
      const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return discrete_state_indexes_;
  }

  /** Returns the geometries of the deformable bodies at reference
   configuration. The geometries have the same order as the registration of
   their corresponding deformable bodies. */
  const std::vector<internal::ReferenceDeformableGeometry<T>>&
  reference_configuration_geometries() const {
    return reference_configuration_geometries_;
  }

  /* Returns the proximity properties of the registered deformable bodies in the
   same order as they are registered. */
  const std::vector<geometry::ProximityProperties>& proximity_properties()
      const {
    return proximity_properties_;
  }

  /** Returns the names of all the registered deformable bodies in the same
   order as the bodies were registered. */
  const std::vector<std::string>& names() const { return names_; }

  // TODO(xuchenhan-tri): Consider changing the underlying data type to
  // std::vector<Matrix3X<T>> for easier parsing.
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
  // TODO(xuchenhan-tri): Implement CloneToDouble() and CloneToAutoDiffXd() and
  //  the corresponding is_cloneable methods.

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
  /* The FemModels owned by this DeformableModel. One per deformable body. */
  std::vector<std::unique_ptr<FemModelBase<T>>> fem_models_{};
  /* The geometries of the deformable bodies at reference configuration. */
  std::vector<internal::ReferenceDeformableGeometry<T>>
      reference_configuration_geometries_{};
  /* The state of each deformable body at reference configuration. This contains
   the same information as the vertex positions in
   reference_configuration_geometries_, but is stored in a more convenient
   format. */
  std::vector<VectorX<T>> model_discrete_states_;
  /* Proximity properties for all registered deformable bodies. */
  std::vector<geometry::ProximityProperties> proximity_properties_{};
  /* Names of all registered deformable bodies. */
  std::vector<std::string> names_{};
  /* OutputPorts. */
  const systems::OutputPort<T>* vertex_positions_port_;
  /* The discrete state indexes for all deformable bodies. */
  std::vector<systems::DiscreteStateIndex> discrete_state_indexes_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::DeformableModel);
