#include "drake/systems/framework/system_html.h"

namespace drake {
namespace systems {

std::string GenerateHtml(const System<double>&, int) {
  return R"""(
<div style='height:400px;' id='myDiagramDiv'>
The implementation of GenerateHtml has been permanently removed from Drake due
to licensing restrictions.
</div>
)""";
}

}  // namespace systems
}  // namespace drake
