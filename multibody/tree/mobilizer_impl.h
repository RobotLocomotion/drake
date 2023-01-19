#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/random.h"
#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* Base class for specific Mobilizer implementations with the number of
generalized positions and velocities resolved at compile time as template
parameters. Also uses CRTP to pick up specific MobilizerType implementations
without virtual function calls, permitting fast operations to be inlined.

This design allows specific mobilizer implementations to work only with
fixed-size Eigen expressions therefore allowing for optimized operations on
fixed-size matrices. In addition, this layer discourages the proliferation
of dynamic-sized Eigen matrices that would otherwise lead to run-time
dynamic memory allocations.

MobilizerImpl also provides a number of size specific methods to retrieve
multibody quantities of interest from caching structures. These are common
to all mobilizer implementations and therefore they live in this class.
Users should not need to interact with this class directly unless they need
to implement a custom Mobilizer class.

Derived class interface
-----------------------
Every derived mobilizer must implement the theory as presented in the class
description for Mobilizer. With one exception, there is complete freedom when
choosing the meaning and ordering of the scalar generalized coordinates q and
generalized speeds v for a mobilizer, provided that the number of v's (nv) is
equal to the number of degrees of freedom provided by the mobilizer.

The exception: if your mobilizer uses a quaternion as all or part of its q's,
the four quaternion elements must come first in order (w xyz) where w is the
scalar element and xyz the vector. This matters when we set the mobilizer to
its zero position -- the base class will set the q's to zero except that if
there is a quaternion it will set the _first_ q to 1 to make the quaternion
(1 0 0 0) which is the identity rotation.

TODO(sherm1) Use the above restriction to normalize the quaternion when
 needed.

Every concrete mobilizer class must implement the following data members and
functions:

static constexpr
  bool kCanRotate, kCanTranslate, kIsFloating, kHasQuaternion

virtuals
  DoCloneToScalar (double, AutoDiffXd, Expression)

inlines (potentially)
  CalcX_FM(q)
  CalcV_FM(q;v) = H_FM(q) v
  CalcA_FM(q,v;v̇) = H_FM(q) v̇ + Hdot_FM(q,v) v
  ProjectF_M(q;F) = H_FMᵀ(q) F
  MapQdotToV(q;q̇) = N(q) q̇
  MapVToQdot(q;v) = N⁺(q) v

These inlines are used in inner-loop computations so need to be very fast.
Typically, H is constant (so Hdot=0) and almost as often it is very sparse, e.g.
H=[0 0 1 0 0 0]ᵀ for a revolute mobilizer or H=I₆ₓ₆ for a floating mobilizer.
And N is most often an identity matrix. Be sure to hand-craft the above
methods to take advantage of those simplifications where possible.

TODO(sherm1) Now that we know at compile time if a mobilizer can't rotate or
 can't translate, that half of X, V, A, and F never changes (and no changes at
 all for a Weld). Take advantage of that by extending the inline API above to
 add "update" functions that modify only the changed parts.

@tparam_default_scalar */
template <typename T, int nq, int nv, template <typename> class MobilizerTypeT>
class MobilizerImpl : public Mobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilizerImpl)

  // Set up compile time mobilizer properties.
  using MobilizerType = MobilizerTypeT<T>;
  static constexpr int kNq = nq;
  static constexpr int kNv = nv;
  static constexpr int kNx = kNq + kNv;
  static constexpr bool kCanRotate = MobilizerType::kCanRotate;
  static constexpr bool kCanTranslate = MobilizerType::kCanTranslate;
  static constexpr bool kIsFloating = MobilizerType::kIsFloating;
  static constexpr bool kHasQuaternion = MobilizerType::kHasQuaternion;

  // As with Mobilizer this the only constructor available for this base class.
  // The minimum amount of information that we need to define a mobilizer is
  // the knowledge of the inboard and outboard frames it connects.
  // Subclasses of %MobilizerImpl are therefore forced to provide this
  // information in their respective constructors.
  MobilizerImpl(const Frame<T>& inboard_frame,
                const Frame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {
    check_compile_time_consistency();  // Just so this gets instantiated.
  }

  // Implement virtuals so the Mobilizer base class can obtain mobilizer
  // properties without knowing the specific type or size.
  int num_positions() const final { return kNq;}
  int num_velocities() const final { return kNv;}
  bool is_floating() const final { return kIsFloating; }
  bool has_quaternion_dofs() const final { return kHasQuaternion; }
  bool can_rotate() const final { return kCanRotate; }
  bool can_translate() const final { return kCanTranslate; }

  // Sets the elements of the `state` associated with this Mobilizer to the
  // _zero_ state.  See Mobilizer::set_zero_state().
  void set_zero_state(const systems::Context<T>&,
                      systems::State<T>* state) const final {
    get_mutable_positions(state) = get_zero_position();
    get_mutable_velocities(state).setZero();
  };

  // Sets the elements of the `state` associated with this Mobilizer to the
  // _default_ state.  See Mobilizer::set_default_state().
  void set_default_state(const systems::Context<T>&,
                         systems::State<T>* state) const final {
    get_mutable_positions(state) = get_default_position();
    get_mutable_velocities(state).setZero();
  };

  // Sets the default position of this Mobilizer to be used in subsequent
  // calls to set_default_state().
  void set_default_position(
      const Eigen::Ref<const Vector<double, kNq>>& position) {
    default_position_.emplace(position);
  }

  // Sets the elements of the `state` associated with this Mobilizer to a
  // _random_ state.  If no random distribution has been set, then `state` is
  // set to the _default_ state.
  void set_random_state(const systems::Context<T>& context,
                        systems::State<T>* state,
                        RandomGenerator* generator) const override {
    if (random_state_distribution_) {
      const Vector<double, kNx> sample = Evaluate(
          *random_state_distribution_, symbolic::Environment{}, generator);
      get_mutable_positions(state) = sample.template head<kNq>();
      get_mutable_velocities(state) = sample.template tail<kNv>();
    } else {
      set_default_state(context, state);
    }
  }

  // Defines the distribution used to draw random samples from this
  // mobilizer, using a symbolic::Expression that contains random variables.
  void set_random_position_distribution(
      const Eigen::Ref<const Vector<symbolic::Expression, kNq>>& position) {
    if (!random_state_distribution_) {
      random_state_distribution_.emplace(
          Vector<symbolic::Expression, kNx>::Zero());
      // Note that there is no `get_zero_velocity()`, since the zero velocity
      // is simply zero for all mobilizers.  Setting the velocity elements of
      // the distribution to zero here therefore maintains the default behavior
      // for velocity.
    }

    random_state_distribution_->template head<kNq>() = position;
  }

  // Defines the distribution used to draw random samples from this
  // mobilizer, using a symbolic::Expression that contains random variables.
  void set_random_velocity_distribution(
      const Eigen::Ref<const Vector<symbolic::Expression, kNv>>& velocity) {
    if (!random_state_distribution_) {
      random_state_distribution_.emplace(
          Vector<symbolic::Expression, kNx>());
      // Maintain the default behavior for position.
      random_state_distribution_->template head<kNq>() = get_zero_position();
    }

    random_state_distribution_->template tail<kNv>() = velocity;
  }

  // TODO(sherm1) Incorporate BodyNode functionality here and get rid of
  //  BodyNode altogether.
  std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node, const Body<T>* body,
      const Mobilizer<T>* mobilizer) const final {
    return std::make_unique<internal::BodyNodeImpl<T, nq, nv>>(parent_node,
                                                               body, mobilizer);
  }

  // These are interfaces to the fast mobilizer specializations of the same name
  // and signature. Each takes the particular mobilizer's q or v state variables
  // as a fixed-size Eigen vector and invokes the specialization with no virtual
  // function calls so that the smaller ones can be inlined.
  math::RigidTransform<T> CalcX_FM(const Vector<T, kNq>& q) const {
    return static_cast<const MobilizerType*>(this)->CalcX_FM(q);
  }

  // These are the implementations of the Mobilizer base class virtual methods,
  // to be used for convenience not for speed. Each of these has to first
  // spelunk the state variables from the context, then invoke the faster
  // inline interface methods above.
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final {
    return CalcX_FM(get_q_ref(context));
  }

 protected:
  // Returns the zero configuration for the mobilizer. This will return
  // q=[0 0 ... 0] except if this mobilizer has a quaternion in which case it
  // returns q=[1 0 ... 0]. Note that we require that the quaternion come
  // first and that its scalar element is first.
  Vector<double, kNq> get_zero_position() const {
    Vector<double, kNq> zero_q = Vector<double, kNq>::Zero();
    if (kHasQuaternion) zero_q[0] = 1.0;  // Known at compile time.
    return zero_q;
  }

  // Returns the default configuration for the mobilizer.  The default
  // configuration is the configuration used to populate the context in
  // MultibodyPlant::SetDefaultContext().
  Vector<double, kNq> get_default_position() const {
    if (default_position_) {
      return *default_position_;
    }
    return get_zero_position();
  }

  // Returns the current distribution governing the random samples drawn
  // for this mobilizer if one has been set.
  const std::optional<Vector<symbolic::Expression, kNx>>&
  get_random_state_distribution() const {
    return random_state_distribution_;
  }

  // @name    Helper methods to retrieve entries from the Context.
  //@{
  // Helper to return a const fixed-size Eigen::VectorBlock referencing the
  // segment in the state vector corresponding to `this` mobilizer's state.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<const VectorX<T>, kNq> get_positions(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().template get_state_segment<kNq>(
        context, this->get_positions_start());
  }

  // Helper to return a mutable fixed-size Eigen::VectorBlock referencing the
  // segment in the state vector corresponding to `this` mobilizer's state.
  // Causes invalidation of at least q-dependent cache entries.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<VectorX<T>, kNq> GetMutablePositions(
      systems::Context<T>* context) const {
    return this->get_parent_tree().template GetMutableStateSegment<kNq>(
        context, this->get_positions_start());
  }

  // Helper variant to return a const fixed-size Eigen::VectorBlock referencing
  // the segment in the `state` corresponding to `this` mobilizer's generalized
  // positions. No cache invalidation occurs.
  // @pre `state` is a valid multibody system State.
  Eigen::VectorBlock<VectorX<T>, kNq> get_mutable_positions(
      systems::State<T>* state) const {
    return this->get_parent_tree().template get_mutable_state_segment<kNq>(
        state, this->get_positions_start());
  }

  // Helper to return a const fixed-size Eigen::VectorBlock referencing the
  // segment in the state vector corresponding to `this` mobilizer's state.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<const VectorX<T>, kNv> get_velocities(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().template get_state_segment<kNv>(context,
        this->get_velocities_start());
  }

  // Helper to return a mutable fixed-size Eigen::VectorBlock referencing the
  // segment in the state vector corresponding to `this` mobilizer's state.
  // Causes invalidation of at least v-dependent cache entries.
  // @pre `context` is a valid multibody system Context.
  Eigen::VectorBlock<VectorX<T>, kNv> GetMutableVelocities(
      systems::Context<T>* context) const {
    return this->get_parent_tree().template GetMutableStateSegment<kNv>(
        context, this->get_velocities_start());
  }

  // Helper variant to return a const fixed-size Eigen::VectorBlock referencing
  // the segment in the `state` corresponding to `this` mobilizer's generalized
  // velocities. No cache invalidation occurs.
  // @pre `state` is a valid multibody system State.
  Eigen::VectorBlock<VectorX<T>, kNv> get_mutable_velocities(
      systems::State<T>* state) const {
    return this->get_parent_tree().template get_mutable_state_segment<kNv>(
        state, this->get_velocities_start());
  }
  //@}

 private:
  // Check compile-time consistency of parameters provided by the derived
  // mobilizer to catch some obvious developer bugs. static_asserts that
  // involve symbols from the derived mobilizer must be deferred to a
  // function definition since the mobilizer type is incomplete at the
  // top level of the MobilizerImpl class. This function must be invoked
  // somewhere in order to get it instantiated but is not otherwise intended
  // to be called since it has no runtime code.
  static void check_compile_time_consistency() {
    static_assert(nq == nv || nq == nv + 1,
        "A mobilizer's number of coordinates nq must be the same "
        "or one greater than the number of velocities nv");
    static_assert(!kIsFloating || nv == 6,
        "A floating mobilizer must have 6 velocities nv");
    static_assert(!kHasQuaternion || nq >= 4,
        "A mobilizer with a quaternion must have at least 4 coordinates nq");
    static_assert(kCanRotate || !(kIsFloating || kHasQuaternion),
        "kCanRotate must be true for Floating or Quaternion mobilizers");
    static_assert(nv > 0 || !(kCanRotate || kCanTranslate),
        "A zero-dof mobilizer can't rotate or translate");
  }

  const Vector<T, kNq>& get_q_ref(const systems::Context<T>& context) const {
    // TODO(sherm1) Do this much faster; should be inlineable.
    Eigen::VectorBlock<const VectorX<T>, kNq> q_block = get_positions(context);
    const T* data = q_block.data();
    return *reinterpret_cast<const Vector<T, kNq>*>(data);
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

  std::optional<Vector<double, kNq>> default_position_{};

  // Note: this is maintained as a concatenated vector so that the evaluation
  // method can share the sampled values of any random variables that are
  // shared between position and velocity.
  std::optional<Vector<symbolic::Expression, kNx>> random_state_distribution_{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
