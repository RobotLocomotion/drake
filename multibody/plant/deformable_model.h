#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/identifier.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/physical_model.h"

namespace drake {
namespace multibody {
namespace internal {

/* Uniquely identifies a deformable body. It is valid before and after
 Finalize(). */
using DeformableBodyId = Identifier<class DeformableBodyTag>;
/* Internally indexes deformable bodies, only used after Finalize(). */
using DeformableBodyIndex = TypeSafeIndex<class DeformableBodyTag>;

/* DeformableModel implements the interface in PhysicalModel and provides the
 functionalities to specify deformable bodies. Unlike rigid bodies, the shape of
 deformable bodies can change in a simulation. Each deformable body is modeled
 as a volumetric mesh with persisting topology, changing vertex positions, and
 an approximated signed distance field. A finite element model is built for each
 registered deformable body that is used to evaluate the dynamics of the body.
 @tparam_double_only */
template <typename T>
class DeformableModel final : public multibody::internal::PhysicalModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableModel)

  /* Construct a DeformableModel to be owned by the given MultibodyPlant.
   @pre plant != nullptr.
   @pre Finalize() has not been called on `plant`. */
  explicit DeformableModel(MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
    DRAKE_DEMAND(!plant_->is_finalized());
  }

  /* Returns the number of deformable bodies registered with this
   DeformableModel. */
  int num_bodies() const { return reference_positions_.size(); }

  // TODO(xuchenhan-tri): Document the minimal requirement on the geometry
  //  instance. For example, it must have a friction proximity property to be
  //  simulated with an MbP that involves contact. Also, move resolution_hint
  //  into the properties of the instance.
  // TODO(xuchenhan-tri): Consider allowing registering deformable bodies with
  //  non-world frames.
  /* Registers a deformable body in `this` DeformableModel with the given
   GeometryInstance. The body is represented in the world frame and simulated
   with FEM with linear elements and a first order quadrature rule that
   integrates linear functions exactly. See FemModel for details. Returns a
   unique identifier for the added geometry.
   @param[in] geometry_instance  The geometry to be registered with the model.
   @param[in] config             The physical properties of deformable body.
   @parame[in] resolution_hint   The parameter that guides the level of mesh
                                 refinement of the deformable geometry. It has
                                 length units (in meters) and roughly
                                 corresponds to a typical edge length in the
                                 resulting mesh for a primitive shape.
   @pre resolution_hint > 0.
   @throws std::exception if Finalize() has been called on the multibody plant
   owning this deformable model. */
  DeformableBodyId RegisterDeformableBody(
      std::unique_ptr<geometry::GeometryInstance> geometry_instance,
      const fem::DeformableBodyConfig<T>& config, double resolution_hint);

  /* Returns the discrete state index of the deformable body identified by the
   given `id`.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet.
   or if no deformable body with the given `id` has been registered in this
   model. */
  systems::DiscreteStateIndex GetDiscreteStateIndex(DeformableBodyId id) const;

  /* Returns the FemModel for the body with `id`.
   @throws exception if no deformable body with `id` is registered with `this`
   %DeformableModel. */
  const fem::FemModel<T>& GetFemModel(DeformableBodyId id) const;

  // TODO(xuchenhan-tri): The use of T over double is not well-reasoned.
  //  Consider whether T is really necessary when we support autodiff in
  //  deformable simulations.
  /* Returns the reference positions of the vertices of the deformable body
   identified by the given `id`.
   The reference positions are represented as a VectorX with 3N values where N
   is the number of vertices. The x-, y-, and z-positions (measured and
   expressed in the world frame) of the j-th vertex are 3j, 3j + 1, and 3j + 2
   in the VectorX.
   @throws std::exception if no deformable body with the given `id` has been
   registered in this model. */
  const VectorX<T>& GetReferencePositions(DeformableBodyId id) const;

  /* Returns the DeformableBodyId of the body with the given body index.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet
   or if index is larger than or equal to the total number of registered
   deformable bodies. */
  DeformableBodyId GetBodyId(DeformableBodyIndex index) const;

  /* Returns the GeometryId of the geometry associated with the body with the
   given `id`.
   @throws std::exception if no body with the given `id` has been registered. */
  geometry::GeometryId GetGeometryId(DeformableBodyId id) const;

 private:
  PhysicalModelPointerVariant<T> DoToPhysicalModelPointerVariant() const final {
    return PhysicalModelPointerVariant<T>(this);
  }

  // TODO(xuchenhan-tri): Implement CloneToDouble() and CloneToAutoDiffXd()
  // and the corresponding is_cloneable methods.

  void DoDeclareSystemResources(MultibodyPlant<T>* plant) final;

  /* Builds a FEM model for the body with `id` with linear tetrahedral elements
   and a single quadrature point. The reference positions as well as the
   connectivity of the elements are given by `mesh`, and physical properties
   such as the material model of the body are given by `config`.
   @throws exception if an FEM model corresponding to `id` already exists. */
  void BuildLinearVolumetricModel(DeformableBodyId id,
                                  const geometry::VolumeMesh<double>& mesh,
                                  const fem::DeformableBodyConfig<T>& config);

  template <template <class, int> class Model>
  void BuildLinearVolumetricModelHelper(
      DeformableBodyId id, const geometry::VolumeMesh<double>& mesh,
      const fem::DeformableBodyConfig<T>& config);

  /* Helper to throw a useful message if a deformable body with the given `id`
   doesn't exist. */
  void ThrowUnlessRegistered(const char* source_method,
                             DeformableBodyId id) const;

  /* The MultibodyPlant that owns `this` DeformableModel. */
  MultibodyPlant<T>* plant_{nullptr};
  /* The positions of each vertex of deformable body at reference configuration.
   */
  std::unordered_map<DeformableBodyId, VectorX<T>> reference_positions_;
  /* The discrete state indexes for all deformable bodies. */
  std::unordered_map<DeformableBodyId, systems::DiscreteStateIndex>
      discrete_state_indexes_;
  std::unordered_map<DeformableBodyId, geometry::GeometryId>
      body_id_to_geometry_id_;
  std::unordered_map<DeformableBodyId, std::unique_ptr<fem::FemModel<T>>>
      fem_models_;
  std::vector<DeformableBodyId> body_ids_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
