#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace lcm {

/** An implementation of DrakeLcmInterface that manipulates LCM messages in
memory, not on the wire. Other than the class name, it is identical to a
`DrakeLcm("memq://")`, i.e., an object constructed with the <a
href="https://lcm-proj.github.io/group__LcmC__lcm__t.html#gaf29963ef43edadf45296d5ad82c18d4b">memq
provider</a>.
*/
class DRAKE_DEPRECATED("2022-03-01", "Use DrakeLcm(\"memq://\") instead.")
DrakeMockLcm : public DrakeLcm {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeMockLcm);

  DrakeMockLcm();
  ~DrakeMockLcm() override;
};

}  // namespace lcm
}  // namespace drake
