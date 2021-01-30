#include "drake/lcm/drake_mock_lcm.h"

namespace drake {
namespace lcm {

DrakeMockLcm::DrakeMockLcm() : DrakeLcm("memq://") {}

DrakeMockLcm::~DrakeMockLcm() {}

}  // namespace lcm
}  // namespace drake
