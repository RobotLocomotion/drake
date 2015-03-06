#include "drake/lcmt_atlas_command.hpp"
#include "QPCommon.h"

class AtlasCommandDriver {
  private:
    int m_num_joints; 
    RobotJointIndexMap joint_index_map;
    drake::lcmt_atlas_command msg;

  public:
    AtlasCommandDriver(JointNames *input_joint_names);
    int dim(void) {
      return 3*m_num_joints;
    }
    void updateGains(AtlasHardwareGains *gains);

    drake::lcmt_atlas_command* encode(double t, QPControllerOutput *qp_output);
    drake::lcmt_atlas_command* encode(double t, QPControllerOutput *qp_output, AtlasHardwareGains *new_gains);
};
