#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <map>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // TODO(amcastro-tri): Assign an infinite mass to the "world" body when rigid
  // body mass properties are implemented.
  RigidBody<T>::Create(this);
}

template <typename T>
void MultibodyTree<T>::CreateTopologyAnalogues() {
  auto& body_topologies = topology_.bodies;
  auto& frame_topologies = topology_.frames;

  body_topologies.reserve(get_num_bodies());
  frame_topologies.reserve(get_num_frames());

  for (auto& body : owned_bodies_) {
    // MultibodyTreeTopology::add_body() generates a BodyIndex and a FrameIndex
    // associated with the corresponding BodyFrame.
    BodyIndex body_index = topology_.add_body();
    FrameIndex frame_index = topology_.bodies[body_index].body_frame;
    DRAKE_DEMAND(static_cast<int>(frame_index) == static_cast<int>(body_index));

    const Frame<T>& body_frame = body->get_body_frame();
    Frame<T>* raw_body_frame_ptr =
        const_cast<Frame<T>*>(&body_frame);

    // All physical frames, including body frames (owned by their respective
    // bodies), are indexed by a FrameIndex within the frames_ array.
    frames_.push_back(raw_body_frame_ptr);

    // Set indexes for later convenient use during topology
    // creation/compilation.
    body->set_index(body_index);
    raw_body_frame_ptr->set_index(frame_index);
  }
  // At this point there should be as many frames as number of bodies since all
  // frames are body frames. Other physical frames are added below.
  DRAKE_ASSERT(topology_.get_num_frames() == get_num_bodies());

  // Create the topology counterparts for owned frames.
  for (auto& frame : owned_frames_) {
    BodyIndex body_index = frame->get_body().get_index();
    FrameIndex frame_index = topology_.add_frame(body_index);
    frames_.push_back(frame.get());
    frame->set_index(frame_index);
  }
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // Crete the topology counterparts to all multibody objects. This process
  // results in the generation of indexes for each multibody object.
  CreateTopologyAnalogues();

  // If the topology is valid it means that this MultibodyTree was already
  // compiled. Since this is an expensive operation, throw an exception to alert
  // users.
  if (topology_is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::Compile() on an already compiled "
            "MultibodyTree.");
  }

  // TODO(amcastro-tri): This is a brief list of operations to be added in
  // subsequent PR's:
  //   - Compile non-T dependent topological information.
  //   - Compute degrees of freedom, array sizes and any other information to
  //     allocate a context and request the required cache entries.
  //   - Setup computational structures (BodyNode based).

  // Give bodies the chance to perform any compile-time setup.
  for (const auto& body : owned_bodies_) {
    body->Compile(*this);
  }

  // Give frames the chance to perform any compile-time setup.
  for (const auto& frame : owned_frames_) {
    frame->Compile(*this);
  }

  validate_topology();
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
