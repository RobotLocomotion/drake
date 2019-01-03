#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/random.h"
#include "drake/common/symbolic.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree_context.h"
#include "drake/multibody/tree/multibody_tree_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/// Base class for specific Mobilizer implementations with the number of
/// generalized positions and velocities resolved at compile time as template
/// parameters. This allows specific mobilizer implementations to only work on
/// fixed-size Eigen expressions therefore allowing for optimized operations on
/// fixed-size matrices. In addition, this layer discourages the proliferation
/// of dynamic-sized Eigen matrices that would otherwise lead to run-time
/// dynamic memory allocations.
/// %MobilizerImpl also provides a number of size specific methods to retrieve
/// multibody quantities of interest from caching structures. These are common
/// to all mobilizer implementations and therefore they live in this class.
/// Users should not need to interact with this class directly unless they need
/// to implement a custom Mobilizer class.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T,
    int compile_time_num_positions, int compile_time_num_velocities>
class MobilizerImpl : public Mobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilizerImpl)

  /// As with Mobilizer this the only constructor available for this base class.
  /// The minimum amount of information that we need to define a mobilizer is
  /// the knowledge of the inboard and outboard frames it connects.
  /// Subclasses of %MobilizerImpl are therefore forced to provide this
  /// information in their respective constructors.
  MobilizerImpl(const Frame<T>& inboard_frame,
                const Frame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {}

  /// Returns the number of generalized coordinates granted by this mobilizer.
  int num_positions() const final { return kNq;}

  /// Returns the number of generalized velocities granted by this mobilizer.
  int num_velocities() const final { return kNv;}

  void set_zero_state(const systems::Context<T>& context,
                      systems::State<T>* state) const final {
    get_mutable_positions(context, state) = get_zero_position();
    get_mutable_velocities(context, state).setZero();
  };

  void set_random_state(const systems::Context<T>& context,
                        systems::State<T>* state,
                        RandomGenerator* generator) const override {
    if (random_state_distribution_) {
      const Vector<double, kNx> sample = Evaluate(
          *random_state_distribution_, symbolic::Environment{}, generator);
      get_mutable_positions(context, state) = sample.template head<kNq>();
      get_mutable_velocities(context, state) = sample.template tail<kNv>();
    } else {
      set_zero_state(context, state);
    }
  }

  /// Defines the distribution used to draw random samples from this
  /// mobilizer, using a symbolic::Expression that contains random variables.
  void set_random_position_distribution(
      const Eigen::Ref<const Vector<symbolic::Expression,
                                    compile_time_num_positions>>& position) {
    if (!random_state_distribution_) {
      random_state_distribution_.emplace(
          Vector<symbolic::Expression, kNx>::Zero());
      // Note that that there is no `get_zero_velocity()`, since the zero
      // velocity is simply zero for all mobilizers.  Setting the velocity
      // elements of the distribution to zero here therefore maintains the
      // default behavior for velocity.
    }

    random_state_distribution_->template head<kNq>() = position;
  }

  /// Defines the distribution used to draw random samples from this
  /// mobilizer, using a symbolic::Expression that contains random variables.
  void set_random_velocity_distribution(
      const Eigen::Ref<const Vector<symbolic::Expression,
                                    compile_time_num_velocities>>& velocity) {
    if (!random_state_distribution_) {
      random_state_distribution_.emplace(
          Vector<symbolic::Expression, kNx>());
      // Maintain the default behavior for position.
      random_state_distribution_->template head<kNq>() = get_zero_position();
    }

    random_state_distribution_->template tail<kNv>() = velocity;
  }

  /// For MultibodyTree internal use only.
  std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node,
      const Body<T>* body, const Mobilizer<T>* mobilizer) const final;

 protected:
  // Handy enum to grant specific implementations compile time sizes.
  // static constexpr int i = 42; discouraged.  See answer in:
  // http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {
    kNq = compile_time_num_positions,
    kNv = compile_time_num_velocities,
    kNx = compile_time_num_positions + compile_time_num_velocities
  };

  /// Returns the zero configuration for the mobilizer.
  virtual Eigen::Matrix<double, kNq, 1> get_zero_position() const {
    return Eigen::Matrix<double, kNq, 1>::Zero();
  }

  /// Returns the current distribution governing the random samples drawn
  /// for this mobilizer.
  const optional<Vector<symbolic::Expression, kNx>>&
  get_random_state_distribution() const {
    return random_state_distribution_;
  }
  /// @name Helper methods to retrieve entries from MultibodyTreeContext.

  /// Helper to return a const fixed-size Eigen::VectorBlock referencing the
  /// segment in the state vector corresponding to `this` mobilizer's state.
  Eigen::VectorBlock<const VectorX<T>, kNq> get_positions(
      const MultibodyTreeContext<T>& context) const {
    return context.template get_state_segment<kNq>(
        this->get_positions_start());
  }

  /// Helper to return a mutable fixed-size Eigen::VectorBlock referencing the
  /// segment in the state vector corresponding to `this` mobilizer's state.
  Eigen::VectorBlock<VectorX<T>, kNq> get_mutable_positions(
      MultibodyTreeContext<T>* context) const {
    return context->template get_mutable_state_segment<kNq>(
        this->get_positions_start());
  }

  /// Returns a mutable reference to the state vector stored in `state` as an
  /// Eigen::VectorBlock<VectorX<T>>.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_vector(
      const systems::Context<T>& context, systems::State<T>* state) const {
    systems::BasicVector<T>& state_vector =
        is_state_discrete(context) ?
        state->get_mutable_discrete_state().get_mutable_vector() :
        dynamic_cast<systems::BasicVector<T>&>(
            state->get_mutable_continuous_state().get_mutable_vector());
    return state_vector.get_mutable_value();
  }

  /// Helper variant to return a const fixed-size Eigen::VectorBlock referencing
  /// the segment in the `state` corresponding to `this` mobilizer's generalized
  /// positions.
  Eigen::VectorBlock<VectorX<T>, kNq> get_mutable_positions(
      const systems::Context<T>& context, systems::State<T>* state) const {
    Eigen::VectorBlock<VectorX<T>> xc =
        get_mutable_state_vector(context, state);
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return xc.nestedExpression().template segment<kNq>(
        this->get_positions_start());
  }

  /// Helper variant to return a const fixed-size Eigen::VectorBlock referencing
  /// the segment in the `state` corresponding to `this` mobilizer's generalized
  /// velocities.
  Eigen::VectorBlock<VectorX<T>, kNv> get_mutable_velocities(
      const systems::Context<T>& context, systems::State<T>* state) const {
    Eigen::VectorBlock<VectorX<T>> xc =
        get_mutable_state_vector(context, state);
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return xc.nestedExpression().template segment<kNv>(
        this->get_velocities_start());
  }

  /// Helper to return a const fixed-size Eigen::VectorBlock referencing the
  /// segment in the state vector corresponding to `this` mobilizer's state.
  Eigen::VectorBlock<const VectorX<T>, kNv> get_velocities(
      const MultibodyTreeContext<T>& context) const {
    return context.template get_state_segment<kNv>(
        this->get_velocities_start());
  }

  /// Helper to return a mutable fixed-size Eigen::VectorBlock referencing the
  /// segment in the state vector corresponding to `this` mobilizer's state.
  Eigen::VectorBlock<VectorX<T>, kNv> get_mutable_velocities(
      MultibodyTreeContext<T>* context) const {
    return context->template get_mutable_state_segment<kNv>(
        this->get_velocities_start());
  }
  /// @}

  /// Helper method to retrieve a const reference to the MultibodyTreeContext
  /// object referenced by `context`.
  /// @throws std::logic_error if `context` is not a MultibodyTreeContext
  /// object.
  static const MultibodyTreeContext<T>& GetMultibodyTreeContextOrThrow(
      const systems::Context<T>& context) {
    // TODO(amcastro-tri): Implement this in terms of
    // MultibodyTree::GetMultibodyTreeContextOrThrow() with additional validity
    // checks.
    const MultibodyTreeContext<T>* mbt_context =
        dynamic_cast<const MultibodyTreeContext<T>*>(&context);
    if (mbt_context == nullptr) {
      throw std::logic_error("The provided systems::Context is not a"
                             "drake::multibody::MultibodyTreeContext.");
    }
    return *mbt_context;
  }

  /// Helper method to retrieve a mutable pointer to the MultibodyTreeContext
  /// object referenced by `context`.
  /// @throws std::logic_error if `context` is not a MultibodyTreeContext
  /// object.
  MultibodyTreeContext<T>& GetMutableMultibodyTreeContextOrThrow(
      systems::Context<T>* context) const {
    // TODO(amcastro-tri): Implement this in terms of
    // MultibodyTree::GetMutableMultibodyTreeContextOrThrow().
    MultibodyTreeContext<T>* mbt_context =
        dynamic_cast<MultibodyTreeContext<T>*>(context);
    if (mbt_context == nullptr) {
      throw std::logic_error("The provided systems::Context is not a "
                             "drake::multibody::MultibodyTreeContext.");
    }
    return *mbt_context;
  }

 private:
  /// Helper that returns `true` if the state of the multibody system is stored
  /// as discrete state.
  static bool is_state_discrete(const systems::Context<T>& context) {
    return GetMultibodyTreeContextOrThrow(context).is_state_discrete();
  }

  // Returns the index in the global array of generalized coordinates in the
  // MultibodyTree model to the first component of the generalized coordinates
  // vector that corresponds to this mobilizer.
  int get_positions_start() const {
    return this->get_topology().positions_start;
  }

  // Returns the index in the global array of generalized velocities in the
  // MultibodyTree model to the first component of the generalized velocities
  // vector that corresponds to this mobilizer.
  int get_velocities_start() const {
    return this->get_topology().velocities_start;
  }

  // Note: this is maintained as a concatenated vector so that the evaluation
  // method can share the sampled values of any random variables that are
  // shared between position and velocity.
  optional<Vector<symbolic::Expression, kNx>> random_state_distribution_{};
};

}  // namespace internal

/// WARNING: This will be removed on or around 2019/03/01.
template <typename T, int A, int B>
using MobilizerImpl
DRAKE_DEPRECATED(
    "This public alias is deprecated, and will be removed around 2019/03/01.")
    = internal::MobilizerImpl<T, A, B>;

}  // namespace multibody
}  // namespace drake
