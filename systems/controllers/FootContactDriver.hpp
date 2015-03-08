#include "drake/foot_contact_estimate_t.hpp"
#include "QPCommon.h"

class FootContactDriver {
  private:
    BodyIdsCache m_body_ids;

  public:
    FootContactDriver(BodyIdsCache body_ids);
    void decode(drake::foot_contact_estimate_t *msg, Ref<VectorXd> &contact_force_detected);
};
