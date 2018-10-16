#pragma once

namespace drake {
namespace geometry {
namespace dev {
namespace render {

/** Specification of render fidelity to use for generating renderings of the
 SceneGraph. Each fidelity will consume an instance of PerceptionProperties.
 Each fidelity level has independent requirements for what properties should be
 set. See the list below for details:

 - low fidelity - see @ref render_engine_vtk_properties "Low-fidelity geometry perception properties"
 - medium fidelity -- not supported yet.
 - high fidelity -- not supported yet.
 */
enum class Fidelity {
  kLow,
  kMedium,
  kHigh
};

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
