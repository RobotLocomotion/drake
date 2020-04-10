#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace lcm {

/**
 * An implementation of DrakeLcmInterface that manipulates LCM messages in
 * memory, not on the wire. Other than the class name, it is identical to a
 * DrakeLcm object constructed with the memq URI, i.e., `DrakeLcm("memq://")`.
 */
class DrakeMockLcm : public DrakeLcm {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeMockLcm);

  DrakeMockLcm();
};

}  // namespace lcm
}  // namespace drake
