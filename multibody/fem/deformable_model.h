#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/physical_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* DeformableModel implements the interface in PhysicalModel and provides the
 functionalities to specify deformable bodies. Unlike rigid bodies, the shape of
 deformable bodies can change in a simulation. Each deformable body is modeled
 as a volumetric mesh with persisting topology, changing vertex positions, and
 an approximated signed distance field.

 The current positions of the vertices of the mesh representing the deformable
 bodies can be queried via the `vertex_positions` output port. The output port
 is an abstract-valued port containing std::vector<VectorX<T>>. There is one
 VectorX for each deformable body registered with the system. The i-th body
 corresponds to the i-th VectorX. The i-th VectorX has 3N values where N is the
 number of vertices in the i-th mesh. For mesh i, the x-, y-, and z-positions
 (measured and expressed in the world frame) of the j-th vertex are 3j, 3j + 1,
 and 3j + 2 in the i-th VectorX from the output port.

 // TODO(xuchenhan-tri): consider if this paragraph is needed.
 The connectivity of the meshes representing the deformable bodies and their
 initial positions can be queried via `reference_configuration_geometries()`
 which returns the geometries of the deformable bodies at the reference
 configuration. The i-th mesh stores the connectivity for body i, which does not
 change throughout the simulation, as well as the initial positions of the
 vertices of the i-th body.

 @tparam_nonsymbolic_scalar */
template <typename T>
class DeformableModel final : public multibody::internal::PhysicalModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableModel)

  /* Construct a DeformableModel to be owned by the given MultibodyPlant.
   @pre plant != nullptr.
   @pre plant is discrete. */
  explicit DeformableModel(MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
    DRAKE_DEMAND(plant_->is_discrete());
  }

  int num_bodies() const { return num_bodies_; }

  // TODO(xuchenhan-tri): Extend the method to also instantiate a new FEM model.
  // TODO(xuchenhan-tri): Extend the method to also support shapes in general.
  /* Adds a deformable body with proximity properties specified by the given
   `proximity_props`. Returns a unique identifier of the newly added deformable
   body.
   @pre `proximity_props` includes the (`material`, `coulomb_friction`) property
   of type CoulombFriction<double>. */
  void RegisterDeformableBody(const geometry::Box& box, std::string name,
                              geometry::ProximityProperties proximity_props);

  /* Returns the discrete state indexes of all the deformable bodies. There is
   one discrete state per deformable body. */
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes()
      const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return discrete_state_indexes_;
  }

  /* Returns the `vertex_positions` output port that reports the current
   positions of the vertices of the mesh representing the deformable bodies.
   This output port is an abstract-valued port containing
   std::vector<Matrix3X<T>>. There is one Matrix3X for each deformable body
   registered with the system. The i-th body corresponds to the i-th Matrix3X.
   For mesh i, the j-th column contains the value of the x-, y-, and z-positions
   (measured and expressed in the world frame) of the j-th vertex in the i-th
   mesh from the output port. */
  const systems::OutputPort<T>& get_vertex_positions_output_port() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return *vertex_positions_port_;
  }

 private:
  // TODO(xuchenhan-tri): Implement CloneToDouble() and CloneToAutoDiffXd() and
  //  the corresponding is_cloneable methods.

  void DoDeclareSystemResources(MultibodyPlant<T>* plant) final;

  /* The MultibodyPlant that owns `this` DeformableModel. */
  MultibodyPlant<T>* plant_{nullptr};
  /* The state of each deformable body at reference configuration. */
  std::vector<VectorX<T>> reference_positions_;
  /* The discrete state indexes for all deformable bodies. */
  std::vector<systems::DiscreteStateIndex> discrete_state_indexes_;

  int num_bodies_{0};
  const systems::OutputPort<T>* vertex_positions_port_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DeformableModel);
