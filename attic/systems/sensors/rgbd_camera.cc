#include "drake/systems/sensors/rgbd_camera.h"

#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_renderer_vtk.h"

namespace drake {
namespace systems {
namespace sensors {

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation, double z_near,
                       double z_far, double fov_y, bool show_window,
                       int width, int height, bool flat_terrain)
    : tree_(tree),
      frame_(RigidBodyFrame<double>()),
      camera_fixed_(true),
      flat_terrain_(flat_terrain),
      color_camera_info_(width, height, fov_y),
      depth_camera_info_(width, height, fov_y),
      X_WB_initial_(
          Eigen::Translation3d(position[0], position[1], position[2]) *
          Eigen::Isometry3d(math::RollPitchYaw<double>(orientation)
                                .ToMatrix3ViaRotationMatrix())),
      renderer_(new RgbdRendererVTK(
          RenderingConfig{width, height, fov_y, z_near, z_far, show_window},
          Eigen::Translation3d(position[0], position[1], position[2]) *
          Eigen::Isometry3d(math::RollPitchYaw<double>(orientation)
                                .ToMatrix3ViaRotationMatrix()) * X_BC_)) {
  InitPorts(name);
  InitRenderer();
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame, double z_near,
                       double z_far, double fov_y, bool show_window,
                       int width, int height, bool flat_terrain)
    : tree_(tree),
      frame_(frame),
      camera_fixed_(false),
      flat_terrain_(flat_terrain),
      color_camera_info_(width, height, fov_y),
      depth_camera_info_(width, height, fov_y),
      renderer_(
          new RgbdRendererVTK(RenderingConfig{width, height, fov_y,
                                              z_near, z_far, show_window},
                              Eigen::Isometry3d::Identity())) {
  InitPorts(name);
  InitRenderer();
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       std::unique_ptr<RgbdRenderer> renderer,
                       bool flat_terrain)
    : tree_(tree),
      frame_(RigidBodyFrame<double>()),
      camera_fixed_(true),
      flat_terrain_(flat_terrain),
      color_camera_info_(renderer->config().width, renderer->config().height,
                         renderer->config().fov_y),
      depth_camera_info_(renderer->config().width, renderer->config().height,
                         renderer->config().fov_y),
      X_WB_initial_(
          Eigen::Translation3d(position[0], position[1], position[2]) *
          Eigen::Isometry3d(math::RollPitchYaw<double>(orientation)
                                .ToMatrix3ViaRotationMatrix())) {
  renderer_ = std::move(renderer);
  InitPorts(name);
  InitRenderer();
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       std::unique_ptr<RgbdRenderer> renderer,
                       bool flat_terrain)
    : tree_(tree),
      frame_(frame),
      camera_fixed_(false),
      flat_terrain_(flat_terrain),
      color_camera_info_(renderer->config().width, renderer->config().height,
                         renderer->config().fov_y),
      depth_camera_info_(renderer->config().width, renderer->config().height,
                         renderer->config().fov_y) {
  renderer_ = std::move(renderer);
  InitPorts(name);
  InitRenderer();
}

void RgbdCamera::InitPorts(const std::string& name) {
  set_name(name);
  const int kVecNum =
      tree_.get_num_positions() + tree_.get_num_velocities();

  state_input_port_ = &this->DeclareInputPort(systems::kVectorValued, kVecNum);

  ImageRgba8U color_image(
    color_camera_info_.width(), color_camera_info_.height());
  color_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageRgba8U(color_image), &RgbdCamera::OutputColorImage);

  ImageDepth32F depth_image(
      depth_camera_info_.width(), depth_camera_info_.height());
  depth_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageDepth32F(depth_image), &RgbdCamera::OutputDepthImage);

  ImageLabel16I label_image(
      color_camera_info_.width(), color_camera_info_.height());
  label_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageLabel16I(label_image), &RgbdCamera::OutputLabelImage);

  camera_base_pose_port_ = &this->DeclareVectorOutputPort(
      rendering::PoseVector<double>(), &RgbdCamera::OutputPoseVector);
}

void RgbdCamera::InitRenderer() {
  // Creates rendering world.
  for (const auto& body : tree_.get_bodies()) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    const int body_id = body->get_body_index();
    auto& body_visual_indices = body_visual_indices_map_[body_id];
    for (const auto& visual : body->get_visual_elements()) {
      optional<VisualIndex> visual_index =
          renderer_->RegisterVisual(visual, body_id);
      if (visual_index) {
        body_visual_indices.push_back(*visual_index);
      }
    }
  }

  if (flat_terrain_)
    renderer_->AddFlatTerrain();
}

const InputPort<double>& RgbdCamera::state_input_port() const {
  return *state_input_port_;
}

const OutputPort<double>&
RgbdCamera::camera_base_pose_output_port() const {
  return *camera_base_pose_port_;
}

const OutputPort<double>&
RgbdCamera::color_image_output_port() const {
  return *color_image_port_;
}

const OutputPort<double>&
RgbdCamera::depth_image_output_port() const {
  return *depth_image_port_;
}

const OutputPort<double>&
RgbdCamera::label_image_output_port() const {
  return *label_image_port_;
}

void RgbdCamera::OutputPoseVector(
    const Context<double>& context,
    rendering::PoseVector<double>* pose_vector) const {
  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, state_input_port_->get_index());

  // Calculates X_WB.
  Eigen::Isometry3d X_WB;
  if (camera_fixed_) {
    X_WB = X_WB_initial_;
  } else {
    // Updates camera pose.
    // TODO(sherm1) Computation of X_WB should be cached since it is needed by
    // the VTK update cache entry.
    const Eigen::VectorXd q =
        input_vector->CopyToVector().head(tree_.get_num_positions());
    KinematicsCache<double> cache = tree_.doKinematics(q);
    X_WB = tree_.CalcFramePoseInWorldFrame(cache, frame_);
  }

  Eigen::Translation<double, 3> trans{X_WB.translation()};
  pose_vector->set_translation(trans);

  Eigen::Quaterniond quat{X_WB.linear()};
  pose_vector->set_rotation(quat);
}

void RgbdCamera::UpdateModelPoses(
    const BasicVector<double>& input_vector) const {
  const Eigen::VectorXd q =
      input_vector.CopyToVector().head(tree_.get_num_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

  if (!camera_fixed_) {
    // Updates camera poses.
    Eigen::Isometry3d X_WB =
      tree_.CalcFramePoseInWorldFrame(cache, frame_);
    renderer_->UpdateViewpoint(X_WB * X_BC_);
  }

  // Updates body poses.
  for (const auto& body : tree_.get_bodies()) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    const auto X_WBody = tree_.CalcBodyPoseInWorldFrame(cache, *body);

    const auto& body_visual_indices =
        body_visual_indices_map_.at(body->get_body_index());
    for (VisualIndex i : body_visual_indices) {
      const auto& visual = body->get_visual_elements()[i];
      const auto X_WV = X_WBody * visual.getLocalTransform();
      renderer_->UpdateVisualPose(X_WV, body->get_body_index(),
                                  RgbdRenderer::VisualIndex(i));
    }
  }
}

void RgbdCamera::OutputColorImage(const Context<double>& context,
                                  ImageRgba8U* color_image) const {
  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, state_input_port_->get_index());

  UpdateModelPoses(*input_vector);
  renderer_->RenderColorImage(color_image);
}

void RgbdCamera::OutputDepthImage(const Context<double>& context,
                                  ImageDepth32F* depth_image) const {
  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, state_input_port_->get_index());

  UpdateModelPoses(*input_vector);
  renderer_->RenderDepthImage(depth_image);
}

void RgbdCamera::OutputLabelImage(const Context<double>& context,
                                  ImageLabel16I* label_image) const {
  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, state_input_port_->get_index());

  UpdateModelPoses(*input_vector);
  renderer_->RenderLabelImage(label_image);
}

RgbdCameraDiscrete::RgbdCameraDiscrete(
    std::unique_ptr<RgbdCamera> camera, double period, bool render_label_image)
    : camera_(camera.get()), period_(period) {
  const auto& color_camera_info = camera->color_camera_info();
  const auto& depth_camera_info = camera->depth_camera_info();

  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(camera));
  input_port_state_ = builder.ExportInput(camera_->state_input_port());

  // Color image.
  const Value<ImageRgba8U> image_color(
      color_camera_info.width(), color_camera_info.height());
  const auto* const zoh_color =
      builder.AddSystem<ZeroOrderHold>(period_, image_color);
  builder.Connect(camera_->color_image_output_port(),
                  zoh_color->get_input_port());
  output_port_color_image_ = builder.ExportOutput(zoh_color->get_output_port());

  // Depth image.
  const Value<ImageDepth32F> image_depth(
      depth_camera_info.width(), depth_camera_info.height());
  const auto* const zoh_depth =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth);
  builder.Connect(camera_->depth_image_output_port(),
                  zoh_depth->get_input_port());
  output_port_depth_image_ = builder.ExportOutput(zoh_depth->get_output_port());

  // Label image.
  if (render_label_image) {
    const Value<ImageLabel16I> image_label(
        color_camera_info.width(), color_camera_info.height());
    const auto* const zoh_label =
        builder.AddSystem<ZeroOrderHold>(period_, image_label);
    builder.Connect(camera_->label_image_output_port(),
                    zoh_label->get_input_port());
    output_port_label_image_ =
        builder.ExportOutput(zoh_label->get_output_port());
  }

  // No need to place a ZOH on pose output.
  output_port_pose_ =
      builder.ExportOutput(camera_->camera_base_pose_output_port());

  builder.BuildInto(this);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
