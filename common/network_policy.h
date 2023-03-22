#pragma once

#include <string_view>

namespace drake {
namespace internal {

/** @addtogroup environment_variables
@{
@defgroup allow_network DRAKE_ALLOW_NETWORK

Users can set the environment variable `DRAKE_ALLOW_NETWORK` to a
colon-separated list to limit which components are permitted network access.

The component values supported by Drake are:
- <b>lcm</b> (see drake::lcm::DrakeLcm)
- <b>meshcat</b> (see drake::geometry::Meshcat)
- <b>package_map</b> (see drake::multibody::PackageMap)
- <b>render_gltf_client</b> (see drake::geometry::MakeRenderEngineVtk())

... as well as the special value <b>none</b>.

For example, to allow only lcm and meshcat, run this command in your terminal:
```
export DRAKE_ALLOW_NETWORK=lcm:meshcat
```

---
Special cases:
- If `DRAKE_ALLOW_NETWORK` is unset or set to the empty string, then all
  networking is allowed.
- If `DRAKE_ALLOW_NETWORK` is set to "none" or a list that contains "none" then
  all networking is denied. This holds true even if other components are listed,
  e.g., "none:lcm" still denies lcm.
@}
*/

/* Returns true iff the given component is allowed to access the network,
based on the current value of the DRAKE_ALLOW_NETWORK environment variable.
@param component is the name to query, which must be alphanumeric lowercase
(underscores allowed) and must not be "none" nor the empty string. */
bool IsNetworkingAllowed(std::string_view component);

}  // namespace internal
}  // namespace drake
