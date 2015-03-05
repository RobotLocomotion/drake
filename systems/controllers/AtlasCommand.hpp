#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_atlas_command.hpp"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class AtlasCommand {
  private:
    static lcm::LCM lcm;
    int m_num_joints; 
    std::vector<int> drake_to_atlas_joint_map;
    drake::lcmt_atlas_command msg;

  public:
    AtlasCommand(const std::vector<std::string>& joint_names, const int atlas_version_number, 
      const Eigen::VectorXd& k_q_p, 
      const Eigen::VectorXd& k_q_i, const Eigen::VectorXd& k_qd_p, const Eigen::VectorXd& k_f_p, 
      const Eigen::VectorXd& ff_qd, const Eigen::VectorXd& ff_qd_d, const Eigen::VectorXd& ff_f_d,
      const Eigen::VectorXd& ff_const);
    void updateGains(const Eigen::VectorXd& k_q_p, const Eigen::VectorXd& k_q_i,const Eigen::VectorXd& k_qd_p, const Eigen::VectorXd& k_f_p, const Eigen::VectorXd& ff_qd, const Eigen::VectorXd& ff_qd_d, const Eigen::VectorXd& ff_f_d, const Eigen::VectorXd& ff_const);
    int dim(void) {
      return 3*m_num_joints;
    }
    void publish(const std::string& channel, double t, const Eigen::VectorXd& x);
};
