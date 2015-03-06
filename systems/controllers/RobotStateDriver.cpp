#include <mex.h>
#include "RobotStateDriver.hpp"

using namespace std;

RobotStateDriver::RobotStateDriver(vector<string> state_coordinate_names) {
  m_num_joints = 0;
  m_num_floating_joints = 0;

  string prefix("base_");
  for (int i=0; i<state_coordinate_names.size(); i++) {
    if (state_coordinate_names[i].compare(0, prefix.size(), prefix) == 0) {
      m_floating_joint_map[state_coordinate_names[i]] = i;
      m_num_floating_joints++;
    }
    else {
      m_joint_map[state_coordinate_names[i]] = i;
      m_num_joints++;
    }
  }

  // this class is assuming an RPY floating base, not quaternion, so let's verify that's what we've got
  map<string,int>::iterator it = m_floating_joint_map.find("base_roll");
  if (it == m_floating_joint_map.end()) {
    mexErrMsgTxt("This method has not yet been updated to support quaternion floating base. It's assuming roll, pitch, yaw for floating base pose, but I couldn't find a 'base_roll' coordinate. \n");
  }
}

void RobotStateDriver::decode(drake::robot_state_t *msg, DrakeRobotState *state) {
  state->t = ((double) msg->utime) / 1000000;

  for (int i=0; i < msg->num_joints; i++) {
    map<string,int>::iterator it = m_joint_map.find(msg->joint_name[i]);
    if (it!=m_joint_map.end()) {
      int index = it->second;
      state->q(index) = msg->joint_position[i];
      state->qd(index) = msg->joint_velocity[i];
    }
  }

  map<string,int>::iterator it;
  it = m_floating_joint_map.find("base_x");
  if (it!=m_floating_joint_map.end()) {
    int index = it->second;
    state->q(index) = msg->pose.translation.x;
    state->qd(index) = msg->twist.linear_velocity.x;
  }
  it = m_floating_joint_map.find("base_y");
  if (it!=m_floating_joint_map.end()) {
    int index = it->second;
    state->q(index) = msg->pose.translation.y;
    state->qd(index) = msg->twist.linear_velocity.y;
  }
  it = m_floating_joint_map.find("base_z");
  if (it!=m_floating_joint_map.end()) {
    int index = it->second;
    state->q(index) = msg->pose.translation.z;
    state->qd(index) = msg->twist.linear_velocity.z;
  }

  Vector4d q;
  q(0) = msg->pose.rotation.w;
  q(1) = msg->pose.rotation.x;
  q(2) = msg->pose.rotation.y;
  q(3) = msg->pose.rotation.z;
  Vector3d rpy = quat2rpy(q);

  Vector3d omega;
  omega(0) = msg->twist.angular_velocity.x;
  omega(1) = msg->twist.angular_velocity.y;
  omega(2) = msg->twist.angular_velocity.z;
  Vector3d rpydot = angularvel2rpydot(rpy,omega);

  it = m_floating_joint_map.find("base_roll");
  if (it!=m_floating_joint_map.end()) {
    int index = it->second;
    state->q(index) = rpy[0];
    state->qd(index) = rpydot[0];
  }

  it = m_floating_joint_map.find("base_pitch");
  if (it!=m_floating_joint_map.end()) {
    int index = it->second; 
    state->q(index) = rpy[1];
    state->qd(index) = rpydot[1];
  }

  it = m_floating_joint_map.find("base_yaw");
  if (it!=m_floating_joint_map.end()) {
    int index = it->second; 
    state->q(index) = rpy[2];
    state->qd(index) = rpydot[2];
  }
  return;
}

