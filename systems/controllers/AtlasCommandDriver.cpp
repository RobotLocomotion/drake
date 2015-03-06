#include "AtlasCommandDriver.hpp"

using namespace std;

AtlasCommandDriver::AtlasCommandDriver(JointNames *input_joint_names) {
  getRobotJointIndexMap(input_joint_names, &joint_index_map);
  m_num_joints = input_joint_names->robot.size();

  msg.num_joints = m_num_joints;
  msg.joint_names.resize(msg.num_joints);

  msg.position.resize(msg.num_joints);
  msg.velocity.resize(msg.num_joints);
  msg.effort.resize(msg.num_joints);

  msg.k_q_p.resize(msg.num_joints);
  msg.k_q_i.resize(msg.num_joints);
  msg.k_qd_p.resize(msg.num_joints);
  msg.k_f_p.resize(msg.num_joints);
  msg.ff_qd.resize(msg.num_joints);
  msg.ff_qd_d.resize(msg.num_joints);
  msg.ff_f_d.resize(msg.num_joints);
  msg.ff_const.resize(msg.num_joints);

  msg.k_effort.resize(msg.num_joints); // only used in sim
  msg.desired_controller_period_ms = 1; // set desired controller rate (ms), only used in sim

  for (int i=0; i<m_num_joints; i++) {
    msg.position[i] = 0.0;
    msg.velocity[i] = 0.0;
    msg.effort[i] = 0.0;

    msg.joint_names[joint_index_map.drake_to_robot[i]] = input_joint_names->drake[i];
    
    msg.k_q_p[joint_index_map.drake_to_robot[i]] = 0;
    msg.k_q_i[joint_index_map.drake_to_robot[i]] = 0;;
    msg.k_qd_p[joint_index_map.drake_to_robot[i]] = 0;;
    msg.k_f_p[joint_index_map.drake_to_robot[i]] = 0;;
    msg.ff_qd[joint_index_map.drake_to_robot[i]] = 0;;
    msg.ff_qd_d[joint_index_map.drake_to_robot[i]] = 0;;
    msg.ff_f_d[joint_index_map.drake_to_robot[i]] = 0;;
    msg.ff_const[joint_index_map.drake_to_robot[i]] = 0;;

    msg.k_effort[joint_index_map.drake_to_robot[i]] = (uint8_t)255; // take complete control of joints (remove BDI control), sim only
  }
}

void AtlasCommandDriver::updateGains(AtlasHardwareGains *gains) {

  if (gains->k_q_p.size() != m_num_joints)
    mexErrMsgTxt("Length of k_q_p must be equal to m_num_joints");
  if (gains->k_q_i.size() != m_num_joints)
    mexErrMsgTxt("Length of k_q_i must be equal to m_num_joints");
  if (gains->k_qd_p.size() != m_num_joints)
    mexErrMsgTxt("Length of k_qd_p must be equal to m_num_joints");
  if (gains->k_f_p.size() != m_num_joints)
    mexErrMsgTxt("Length of k_f_p must be equal to m_num_joints");
  if (gains->ff_qd.size() != m_num_joints)
    mexErrMsgTxt("Length of ff_qd must be equal to m_num_joints");
  if (gains->ff_qd_d.size() != m_num_joints)
    mexErrMsgTxt("Length of ff_qd_d must be equal to m_num_joints");
  if (gains->ff_f_d.size() != m_num_joints)
    mexErrMsgTxt("Length of ff_f_d must be equal to m_num_joints");
  if (gains->ff_const.size() != m_num_joints)
    mexErrMsgTxt("Length of ff_const must be equal to m_num_joints");
  
  for (int i=0; i<m_num_joints; i++) {
    msg.k_q_p[joint_index_map.drake_to_robot[i]] = gains->k_q_p[i];
    msg.k_q_i[joint_index_map.drake_to_robot[i]] = gains->k_q_i[i];
    msg.k_qd_p[joint_index_map.drake_to_robot[i]] = gains->k_qd_p[i];
    msg.k_f_p[joint_index_map.drake_to_robot[i]] = gains->k_f_p[i];
    msg.ff_qd[joint_index_map.drake_to_robot[i]] = gains->ff_qd[i];
    msg.ff_qd_d[joint_index_map.drake_to_robot[i]] = gains->ff_qd_d[i];
    msg.ff_f_d[joint_index_map.drake_to_robot[i]] = gains->ff_f_d[i];
    msg.ff_const[joint_index_map.drake_to_robot[i]] = gains->ff_const[i];
  }
}

drake::lcmt_atlas_command* AtlasCommandDriver::encode(double t, QPControllerOutput *qp_output) {
  return encode(t, qp_output, nullptr);
}

drake::lcmt_atlas_command* AtlasCommandDriver::encode(double t, QPControllerOutput *qp_output, AtlasHardwareGains *new_gains) {
  // Copy data from the given qp_output into the stored LCM message object. If
  // new_gains is NULL, then the existing hardware gains will be used.
  // Otherwise the new gains will be copied in as well.

  msg.utime = (long)(t*1000000);
  int j;
  for (int i=0; i < m_num_joints; i++) {
    j = joint_index_map.drake_to_robot[i];
    msg.position[j] = qp_output->q_ref(i);
    msg.velocity[j] = qp_output->qd_ref(i);
    msg.effort[j] = qp_output->u(i);
  }

  if (new_gains) {
    updateGains(new_gains);
  }
  return &msg;
}
