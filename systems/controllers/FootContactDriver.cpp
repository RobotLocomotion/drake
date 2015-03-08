#include "FootContactDriver.hpp"

FootContactDriver::FootContactDriver(BodyIdsCache body_ids) {
  m_body_ids = body_ids;
}

void FootContactDriver::decode(drake::foot_contact_estimate_t *msg, Ref<VectorXd> &contact_force_detected) {
  contact_force_detected(m_body_ids.r_foot) = msg->right_contact > 0.5;
  contact_force_detected(m_body_ids.l_foot) = msg->left_contact > 0.5;
}



