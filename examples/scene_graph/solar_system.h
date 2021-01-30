#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

namespace examples {
namespace solar_system {

// TODO(SeanCurtis-TRI): When textures are available, modify this so that planet
// rotations become apparent as well (and not just revolutions).
/** A model of an orrery -- a simple mechanical model of the solar system.

 The orrery contains one sun and multiple orbiting bodies: two planets (Earth
 and Mars) each with one moon and multiple satellites for Earth. The idea is
 to represent each type of visual geometry in some form. The orrery is
 articulated by placing the _frame_ for each body at its parent's origin, and
 then displacing the geometry from that origin to its orbital distance. Then
 each orbiting frame has a single degree of freedom: its angular position
 around its axis of rotation.

 - The sun is stationary -- an anchored geometry.
 - Earth orbits on the xy-plane. Its moon (Luna) revolves around the earth on
   a different arbitrary plane (illustrating transform compositions).
 - Three satellites (Convex, Box, and Capsule) revolve around Earth in the same
   way as Luna but at different relative angular positions around their axis of
   rotation.
 - Mars orbits the sun at a farther distance on a plane that is tilted off of
   the xy-plane. Its moon (Phobos) orbits around Mars on a plane parallel to
   Mars's orbital plane, but in the opposite direction.
 - Mars has been given an arbitrary set of rings posed askew. The rings are
   declared as a child of mars's geometry.

 This system illustrates the following features:

 1. Registering anchored geometry.
 2. Registering frames as children of other frames.
 3. Allocating and calculating the FramePoseVector output for visualization.
 4. Exercise all supported SceneGraph geometries in an illustration context.

 Illustration of the orrery:

 Legend:
     Frames   | Meaning
 :-----------:|:--------------------
     S        | The sun's frame
     E - ◯    | Earth's frame
     M - ◍    | Mars's frame
     L - ◑    | Luna's (Earth's moon) frame
     P - ●    | Phobos's (Mars's moon) frame
     Oᵢ       | Body's orbit in a circular path¹

 ¹ `i ∈ {e, m, l, p}` for Earth, Mars, Luna, and Phobos, respectively.

  Pose Symbol | Meaning
 :-----------:|:--------------------
    `X_SOₑ`   | Earth's orbit Oₑ relative to S
    `X_OₑE`   | Earth's frame E relative to its orbit Oₑ
    `X_OₑOₗ`  | Luna's orbit Oₗ relative to Earth's Oₑ
    `X_OₗL`   | Luna's geometry L relative to its orbit Oₗ
    `X_SOₘ`   | Mars's orbit Oₘ relative to S
    `X_OₘM`   | Mars's frame M relative to its orbit Oₘ
    `X_OₘOₚ`  | Phobos's orbit Oₚ relative to Mars's orbit Oₘ
    `X_OₚP`   | Phobos's frame P relative to its orbit Oₚ
    `X_MR`    | Mars's rings R relative to Mars's frame M (not shown in diagram)

```
    X_OₑE  X_OₗL                          X_OₘM  X_OₚP
       ↓   ↓                                ↓   ↓
       E   L         ▁▁▁                    M   P
       ◯   ◑        ╱   ╲                   ◍   ●
X_OₑOₗ→├───┘       │  S  │         X_OₘOₚ → ├───┘
       │            ╲▁▁▁╱                   │
       │              │                     │
       └──────────────┤ ← X_SOₑ             │
                      └─────────────────────┘
                      ↑
                     X_SOₘ
```

 The frame of orbit's origin lies at the circle's center and it's z-axis is
 perpendicular to the plane of the circle.

 @tparam_double_only
*/
template <typename T>
class SolarSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolarSystem)

  explicit SolarSystem(geometry::SceneGraph<T>* scene_graph);
  ~SolarSystem() override = default;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;

  geometry::SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  // The default leaf system zeros out all of the state as "default" behavior.
  // This subverts that (as a quick test) to make sure that's the source of my
  // problems. The long term solution is to make sure SetDefaultState uses the
  // model values if they exist (and zeros otherwise).
  // TODO(SeanCurtis-TRI): Kill this override when LeafSystem::SetDefaultState()
  // pulls from the models instead of simply zeroing things out.
  void SetDefaultState(const systems::Context<T>&,
                       systems::State<T>*) const override;

 private:
  // Allocate all of the geometry.
  void AllocateGeometry(geometry::SceneGraph<T>* scene_graph);

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const MyContext& context,
                           geometry::FramePoseVector<T>* poses) const;

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
  int geometry_pose_port_{-1};

  // Solar system specification
  const int kBodyCount = 7;
  // The ids for each celestial body frame
  std::vector<geometry::FrameId> body_ids_;
  // The axes around each body revolves (expressed in its parent's frame)
  std::vector<Vector3<double>> axes_;
  // The translational offset of each body from its parent frame
  std::vector<math::RigidTransformd> body_offset_;
};

}  // namespace solar_system
}  // namespace examples
}  // namespace drake
