#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace dev {
/**
 * To use RemoteTreeViewerWrapper, please download director from source
 * https://github.com/RobotLocomotion/director, and build it from the source.
 * The drake-visualizer binary shipped with Drake does NOT support Remote Tree
 * Viewer.
 */
class RemoteTreeViewerWrapper {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RemoteTreeViewerWrapper)

  /**
   * If @p lcm is passed in, it will be aliased and used for all publish calls.
   * If nullptr, a new DrakeLcm instance will be made instead.
   * The lifespan of @p lcm needs to be longer than this instance.
   */
  explicit RemoteTreeViewerWrapper(drake::lcm::DrakeLcm* lcm = nullptr);

  ~RemoteTreeViewerWrapper() = default;

  void PublishPointCloud(const Eigen::Matrix3Xd& pts,
                         const std::vector<std::string>& path,
                         const std::vector<std::vector<double>>& color = {
                             {1.0, 0.0, 1.0}});
  void PublishLine(const Eigen::Matrix3Xd& pts,
                   const std::vector<std::string>& path,
                   const Eigen::Ref<const Eigen::Vector4d>& color =
                       Eigen::Vector4d(1, 0, 0, 1));
  void PublishRawMesh(const Eigen::Matrix3Xd& verts,
                      const std::vector<Eigen::Vector3i>& tris,
                      const std::vector<std::string>& path);
  void PublishArrow(const Eigen::Ref<const Eigen::Vector3d>& start,
                    const Eigen::Ref<const Eigen::Vector3d>& end,
                    const std::vector<std::string>& path, double radius,
                    double head_radius, double head_length,
                    const Eigen::Ref<const Eigen::Vector4d>& color =
                        Eigen::Vector4d(1, 0, 0, 1));

  /**
   * Publishes the visual (or collision, if `visual` is false) geometries of
   * `tree` for the configuration `q` as children of `path`. Any geometries that
   * do not specify a color or texture will use `color`.
   */
  void PublishRigidBodyTree(const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q,
                            const Eigen::Vector4d& color,
                            const std::vector<std::string>& path,
                            bool visual = true);
  void PublishRigidBody(const RigidBodyTree<double>& tree, int body_index,
                        const Eigen::Affine3d& tf, const Eigen::Vector4d& color,
                        const std::vector<std::string>& path,
                        bool visual = true);
  void PublishGeometry(const DrakeShapes::Geometry& geometry,
                       const Eigen::Affine3d& tf, const Eigen::Vector4d& color,
                       const std::vector<std::string>& path);

  /**
   * Updates the poses of all geometries in `path`, based on the poses of the
   * corresponding geometries in `tree` for the configuration `q`. For this
   * method to have an effect, `PublishRigidBodyTree()` must be called first
   * with the same `tree`, `path`, and `visual` arguments.
   */
  void UpdateRigidBodyTree(const RigidBodyTree<double>& tree,
                           const Eigen::VectorXd& q,
                           const std::vector<std::string>& path,
                           bool visual = true);

 private:
  drake::lcm::DrakeLcm* lcm_{};
  std::unique_ptr<drake::lcm::DrakeLcm> owned_lcm_{};
};
}  // namespace dev
}  // namespace manipulation
}  // namespace drake
