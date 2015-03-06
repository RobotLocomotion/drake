#include "controlUtil.h"
#include "drake/robot_state_t.hpp"

class RobotStateDriver {
  private:
    int m_num_joints;
    int m_num_floating_joints;
    std::map<std::string,int> m_joint_map;
    std::map<std::string,int> m_floating_joint_map;

  public:
    RobotStateDriver(std::vector<std::string> state_coordinate_names);
    void decode(drake::robot_state_t *msg, DrakeRobotState *state);
};
