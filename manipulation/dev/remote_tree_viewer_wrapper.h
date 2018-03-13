#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace dev {
class RemoteTreeViewerWrapper {
  public:
    RemoteTreeViewerWrapper();
    void publishPointCloud(
            const Eigen::Matrix3Xd& pts,
            const std::vector<std::string>& path,
            const std::vector<std::vector<double>>& color = {{1.0, 0.0, 1.0}});
    void publishLine(
            const Eigen::Matrix3Xd& pts,
            const std::vector<std::string>& path);
    void publishRawMesh(
            const Eigen::Matrix3Xd& verts,
            const std::vector<Eigen::Vector3i>& tris,
            const std::vector<std::string>& path);
    void publishRigidBodyTree(
            const RigidBodyTree<double>& tree,
            const Eigen::VectorXd& q,
            const Eigen::Vector4d& color,
            const std::vector<std::string>& path,
            bool visual = true);
    void publishRigidBody(
            const RigidBody<double>& body,
            const Eigen::Affine3d& tf,
            const Eigen::Vector4d& color,
            const std::vector<std::string>& path);
    void publishGeometry(
            const DrakeShapes::Geometry& geometry,
            const Eigen::Affine3d& tf,
            const Eigen::Vector4d& color,
            const std::vector<std::string>& path);

  private:
    drake::lcm::DrakeLcm lcm_;
};
}  // namespace dev
}  // namespace manipulation
}  // namespace drake
