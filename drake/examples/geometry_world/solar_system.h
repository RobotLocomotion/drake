#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

namespace examples {
namespace solar_system {

// TODO(SeanCurtis-TRI): When textures are available, modify this so that planet
// rotations become apparent as well (and not just revolutions).
/** A model of an orrery -- a simple mechanical model of the solar system.

 The orrery contains one sun and four orbiting bodies: two planets (Earth and
 Mars) each with one moon. The orrery is articulated by placing the _frame_ for
 each body at its parent's origin, and then displacing the geometry from that
 origin to its orbital distance. Then each orbiting frame has a single degree of
 freedom: its angular position around its axis of rotation.

 - The sun is stationary -- an anchored geometry.
 - Earth orbits on the xy-plane. Its moon (Luna) revolves around the earth on an
 different arbitrary plane (illustrating transform compositions).
 - Mars orbits the sun at a farther distance on a plane that is tilted off of
 the xy-plane. Its moon (Phobos) orbits around Mars on a plane parallel to
 Mars's orbital plane.

 This system illustrates the following features:

 1. Registering anchored geometry.
 2. Registering frames as children of other frames.
 3. Creating a fixed FrameIdVector output.
 4. Updating the context-dependent FramePoseVector output.

 Illustration of the orrery:

 Legend:
     Body Symbol | Meaning
    :-----------:|:--------------------
        E - ◯    | Earth
        L - ◑    | Luna (Earth's moon)
        M - ◍    | Mars
        P - ●    | Phobos (Mars's moon)

    Frame Symbol | Meaning
    :-----------:|:--------------------
        X_SE     | Earth's frame (centered on the sun) (X_SE == X_SM)
        X_SM     | Mar's frame (centered on the sun)
        X_EL     | Luna's frame (centered on the Earth)
        X_MP     | Phobos's frame (centered on Mars)
        X_EG     | The fixed offset of Earth's geometry from X_SE (at Earth's orbit radius).
        X_LG     | The fixed offset of Luna's geometry from X_EL (at Luna's orbit radius).
        X_MG     | The fixed offset of Mars's geometry from X_SM (at Mars's orbit radius).
        X_PG     | The fixed offset of Phobos's geometry from X_MP (at Phobos's orbit radius).

```
    X_EG X_LG                            X_MG X_PG
      ↓   ↓                                ↓   ↓
      E   L         ▁▁▁                    M   p
      ◯   ◑        ╱   ╲                   ◍   ●
X_EL →├───┘       │  S  │           X_MP → ├───┘
      │            ╲▁▁▁╱                   │
      │              │                     │
      └──────────────┴─────────────────────┘
                     ↑
                     X_SE, X_SM
```

 @tparam T The vector element type, which must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double */
template <typename T>
class SolarSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolarSystem)

  explicit SolarSystem(geometry::GeometrySystem<T>* geometry_system);
  ~SolarSystem() override = default;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;

  geometry::SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& get_geometry_id_output_port() const;
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  // The default leaf system zeros out all of the state as "default" behavior.
  // This subverts that (as a quick test) to make sure that's the source of my
  // problems. The long term solution is to make sure SetDefaultState uses the
  // model values if they exist (and zeros otherwise).
  // TODO(SeanCurtis-TRI): Kill this override when LeafSystem::SetDefaultState()
  // pulls from the models instead of simply zeroing things out.
  void SetDefaultState(const systems::Context<T>&,
                       systems::State<T>*) const override;

 protected:
  // No inputs implies no feedthrough; this makes it explicit.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 private:
  // Allocate all of the geometry.
  void AllocateGeometry(geometry::GeometrySystem<T>* geometry_system);

  // Allocate the frame pose set output port value.
  geometry::FramePoseVector<T> AllocateFramePoseOutput(
      const MyContext& context) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const MyContext& context,
                           geometry::FramePoseVector<T>* poses) const;

  // Allocate the id output.
  geometry::FrameIdVector AllocateFrameIdOutput(const MyContext& context) const;
  // Calculate the id output.
  void CalcFrameIdOutput(const MyContext& context,
                         geometry::FrameIdVector* id_set) const;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  static const systems::BasicVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const systems::BasicVector<T>&>(cstate.get_vector());
  }

  static systems::BasicVector<T>& get_mutable_state(MyContinuousState* cstate) {
    return dynamic_cast<systems::BasicVector<T>&>(cstate->get_mutable_vector());
  }

  static const systems::BasicVector<T>& get_state(const MyContext& context) {
    return get_state(context.get_continuous_state());
  }

  // Geometry source identifier for this system to interact with geometry system
  geometry::SourceId source_id_{};

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};

  // Solar system specification
  const int kBodyCount = 4;
  // The ids for each celestial body frame
  std::vector<geometry::FrameId> body_ids_;
  // The axes around each body revolves (expressed in its parent's frame)
  std::vector<Vector3<double>> axes_;
  // The translational offset of each body from its parent frame
  std::vector<Isometry3<double>> body_offset_;
};

}  // namespace solar_system
}  // namespace examples
}  // namespace drake
