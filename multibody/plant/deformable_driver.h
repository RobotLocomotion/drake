#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/fem/discrete_time_integrator.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* DeformableDriver is responsible for computing dynamics information about all
 deformable bodies. It works in tandem with a DeformableModel and a
 DiscreteUpdateManager that are provided at construction time. The deformable
 model informs the driver of modeling choices of the deformable bodies
 such as its Finite Element Model. The discrete update manager consumes the
 results of the computation performed by the driver and also provides
 information about the result of the world that the deformable bodies are
 interacting with. In particular, the manager provides access to MultibodyPlant.
 @tparam_double_only */
template <typename T>
class DeformableDriver : public ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableDriver)

  /* Constructs a deformable driver that solves for the dynamics of the given
   `deformable_model`. The newly constructed driver is used in the given
   `manager` to perform discrete updates. The given `deformable_model` and
   `manager` must outlive this driver.
   @pre deformable_model != nullptr.
   @pre manager != nullptr. */
  DeformableDriver(const DeformableModel<T>* deformable_model,
                   const DiscreteUpdateManager<T>* manager);

  ~DeformableDriver();

  // TODO(xuchenhan-tri): Implement CloneToDouble() and allow cloning to double.
  bool is_cloneable_to_double() const final { return false; }
  bool is_cloneable_to_autodiff() const final { return false; }
  bool is_cloneable_to_symbolic() const final { return false; }

  /* Declares cache entries used by this DeformableDriver through the given
   manager.
   @pre `manager` is not nullptr and points to the same DiscreteUpdateManager
   provided at construction. */
  void DeclareCacheEntries(DiscreteUpdateManager<T>* manager);

  /* Evaluates the velocities of all participating dofs. The dofs are ordered
   in increasing order of deformable body indexes. That is, the participating
   dofs of body 0 come first, followed by participating dofs of body 1 and so
   on. Within a single body, the dofs are ordered according to
   their associated vertex index. */
  const VectorX<T>& EvalParticipatingVelocities(
      const systems::Context<T>& context) const;

 private:
  friend class DeformableDriverTest;
  friend class DeformableDriverContactTest;
  friend class DeformableDriverMultiplexerTest;

  /* Struct used to conglomerate the indexes of cache entries declared by
   the manager. */
  struct CacheIndexes {
    /* Per body cache entries indexed by DeformableBodyIndex. */
    std::vector<systems::CacheIndex> fem_states;
    std::vector<systems::CacheIndex> free_motion_fem_states;
    std::vector<systems::CacheIndex> next_fem_states;
    std::vector<systems::CacheIndex> fem_solver_scratches;
    systems::CacheIndex deformable_contact;
    std::vector<systems::CacheIndex> dof_permutations;
    systems::CacheIndex participating_velocity_mux;
    systems::CacheIndex participating_velocities;
  };

  /* This private nested class acts both as a multiplexer and a demultiplexer --
   it combines multiple Eigen vectors into a single stacked vector and it also
   splits an Eigen vector into multiple vectors. */
  class Multiplexer {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Multiplexer);

    /* Create an empty Multiplexer. */
    Multiplexer() = default;

    /* Constructs a Multiplexer that combines and splits vectors of the given
     sizes.
     @pre `sizes` is not empty and each entry is non-negative. */
    explicit Multiplexer(std::vector<int> sizes);

    /* The number of vectors to be multiplexed. */
    int num_vectors() const { return sizes_.size(); }

    /* Combines the given vectors into a single vector.
     @throws std::exception if the sizes of `inputs` aren't compatible with the
     sizes provided at construction. */
    VectorX<T> Multiplex(std::vector<VectorX<T>>&& inputs) const;

    /* Splits the given vector into multiple vectors and returns the one with
     the given `index`.
     @throws std::exception if the size of `input` is not the sum of sizes
     provided at construction.
     @throws std::exception if index is not in [0, num_vectors).
     @returns a copy of the indexed vector. */
    VectorX<T> Demultiplex(const VectorX<T>& input, int index) const;

   private:
    std::vector<int> sizes_;
    std::vector<int> offsets_;
    /* The sum over `sizes_`. */
    int num_entries_{0};
  };

  /* Copies the state of the deformable body with `id` in the given `context`
   to the `fem_state`.
   @pre fem_state != nullptr and has size compatible with the state of the
        deformable body with the given `index`.
   @pre `index` is valid and less than the number of deformable bodies. */
  void CalcFemState(const systems::Context<T>& context,
                    DeformableBodyIndex index,
                    fem::FemState<T>* fem_state) const;

  /* Eval version of CalcFemState(). */
  const fem::FemState<T>& EvalFemState(const systems::Context<T>& context,
                                       DeformableBodyIndex index) const;

  /* Given the state of the deformable body with `index` in the given `context`,
   computes its "free motion" state (the state the body would have at the next
   time step in the absense of contact or constraints).
   @pre fem_state_star != nullptr and is compatible with the state of the
   deformable body with the given `index`. */
  void CalcFreeMotionFemState(const systems::Context<T>& context,
                              DeformableBodyIndex index,
                              fem::FemState<T>* fem_state_star) const;

  /* Eval version of CalcFreeMotionFemState(). */
  const fem::FemState<T>& EvalFreeMotionFemState(
      const systems::Context<T>& context, DeformableBodyIndex index) const;

  /* Given the state of the deformable body with `index` in the given `context`,
   computes the state of the deformable body at the next time step.
   @note The state of the deformable body will the same as the "free motion"
         state in the absense of contact or constraints.
   @pre next_fem_state != nullptr and is compatible with the state of
        the deformable body with the given `index`. */
  void CalcNextFemState(const systems::Context<T>& context,
                        DeformableBodyIndex index,
                        fem::FemState<T>* next_fem_state) const;

  /* Eval version of CalcNextFemState(). */
  const fem::FemState<T>& EvalNextFemState(const systems::Context<T>& context,
                                           DeformableBodyIndex index) const;

  /* Computes the contact information for all registered deformable bodies
   @pre The geometry query input port of the MultibodyPlant that owns the
        manager associated with this DeformableDriver is connected.
   @pre result != nullptr. */
  void CalcDeformableContact(
      const systems::Context<T>& context,
      geometry::internal::DeformableContact<T>* result) const;

  /* Eval version of CalcDeformableContact(). */
  const geometry::internal::DeformableContact<T>& EvalDeformableContact(
      const systems::Context<T>& context) const;

  /* Computes the partial permutation that maps degrees of freedom of the
   deformable body with the given `index` to degrees of freedom that belong to
   vertices of the body that participate in contact.
   @pre result != nullptr. */
  void CalcDofPermutation(
      const systems::Context<T>& context, DeformableBodyIndex index,
      contact_solvers::internal::PartialPermutation* result) const;

  /* Eval version of CalcDofPermutation(). */
  const contact_solvers::internal::PartialPermutation& EvalDofPermutation(
      const systems::Context<T>& context, DeformableBodyIndex index) const;

  /* Computes the multiplexer for participating velocities for all bodies.
   @pre result != nullptr. */
  void CalcParticipatingVelocityMultiplexer(const systems::Context<T>& context,
                                            Multiplexer* result) const;

  /* Eval version of CalcParticipatingVelocityMultiplexer(). */
  const Multiplexer& EvalParticipatingVelocityMultiplexer(
      const systems::Context<T>& context) const;

  /* Calc version of EvalParticipatingVelocities().
   @pre result != nullptr. */
  void CalcParticipatingVelocities(const systems::Context<T>& context,
                                   VectorX<T>* result) const;

  CacheIndexes cache_indexes_;
  /* Modeling information about all deformable bodies. */
  const DeformableModel<T>* const deformable_model_;
  const DiscreteUpdateManager<T>* const manager_;
  /* The integrator used to advance deformable body free motion states in time.
   */
  std::unique_ptr<fem::internal::DiscreteTimeIntegrator<T>> integrator_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
