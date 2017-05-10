#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;
template<typename T> class BodyNode;

/// %Mobilizer is a fundamental object within Drake's multibody engine used to
/// specify the allowed motions between two Frame objects within a
/// MultibodyTree. Specifying the allowed motions between two Frame objects
/// effectively also specifies a kinematic relation between the two bodies
/// associtated with those two frames. Consider the following example to build a
/// simple pendulum system:
///
/// @code
/// MultibodyTree<double> model;
/// // ... Code here to setup quantities below as mass, com, X_BP, etc. ...
/// const Body<double>& pendulum =
///   model.AddBody<RigidBody>(SpatialInertia<double>(mass, com, unit_inertia));
/// // We will connect the pendulum body to the world frame using a
/// // RevoluteMobilizer. To do so we define a pin frame P rigidly attached to
/// // the pendulum body.
/// FixedOffsetFrame<double>& pin_frame =
///   model.AddFrame<FixedOffsetFrame>(
///     pendulum.get_body_frame(),
///     X_BP /* pose of pin frame P in body frame B */);
/// // The mobilizer connects the world frame and the pin frame effetively
/// // adding the single degree of freedom describing this system.
/// const RevoluteMobilizer<double>& revolute_mobilizer =
///   model.AddMobilizer<RevoluteMobilizer>(
///     model.get_world_body().get_body_frame(), /* inboard frame */
///     pin_frame, /* outboard frame */
///     Vector3d::UnitZ() /* revolute axis in this case */));
/// @endcode
///
/// A %Mobilizer induces a tree structure within a MultibodyTree
/// model, connecting an inboard frame to an outboard frame. Every time a
/// %Mobilizer is added to a MultibodyTree (using the
/// MultibodyTree::AddMobilizer() method), a number of degrees of
/// freedom associated with the particular type of %Mobilizer are added to the
/// multibody system. In the example above for the single pendulum, adding a
/// RevoluteMobilizer has two purposes:
/// - It defines the tree structure of the model. World is the "parent" body
///   while "pendulum" is the "child" body in the MultibodyTree.
/// - It informs the MultibodyTree of the degress of freedom granted by the
///   revolute mobilizer between the two frames it connects.
///
/// %Mobilizer is an asbtract base class defining the minimum functionality that
/// derived %Mobilizer objects must implement in order to fully define the
/// kinematic relationship between the two frames they connect.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Mobilizer : public MultibodyTreeElement<Mobilizer<T>, MobilizerIndex> {
 public:
  /// This constructor enforces the minimum requirement to any mobilizer's
  /// constructor; we must specify the inboard and outboard frames that `this`
  /// mobilizer connects.
  Mobilizer(const Frame<T>& inboard_frame,
            const Frame<T>& outboard_frame) :
      inboard_frame_(inboard_frame), outboard_frame_(outboard_frame) {}

  /// Returns the number of generalized coordinates granted by this mobilizer.
  virtual int get_num_positions() const = 0;

  /// Returns the number of generalized velocities granted by this mobilizer.
  virtual int get_num_velocities() const = 0;

  /// Returns a constant reference to the inboard frame.
  const Frame<T>& get_inboard_frame() const {
    return inboard_frame_;
  }

  /// Returns a constant reference to the outboard frame.
  const Frame<T>& get_outboard_frame() const {
    return outboard_frame_;
  }

  /// Returns a constant reference to the body associated with `this`
  /// mobilizer's inboard frame.
  const Body<T>& get_inboard_body() const {
    return get_inboard_frame().get_body();
  }

  /// Returns a constant reference to the body associated with `this`
  /// mobilizer's outboard frame.
  const Body<T>& get_outboard_body() const {
    return get_outboard_frame().get_body();
  }

 private:
  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each mobilizer retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.mobilizers[this->get_index()];
  }

  const Frame<T>& inboard_frame_;
  const Frame<T>& outboard_frame_;
  MobilizerTopology topology_;
};

}  // namespace multibody
}  // namespace drake
