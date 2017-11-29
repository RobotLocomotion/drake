#include "drake/systems/sensors/rgbd_camera.h"

#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

namespace {

// TODO(kunimatsu-tri) Add support for the arbitrary image size
constexpr int kImageWidth = 640;  // In pixels
constexpr int kImageHeight = 480;  // In pixels

}  // namespace

// Note that if `depth_image` holds any pixels that have NaN, the converted
// points will aslo become NaN.
void RgbdCamera::ConvertDepthImageToPointCloud(const ImageDepth32F& depth_image,
                                               const CameraInfo& camera_info,
                                               Eigen::Matrix3Xf* point_cloud) {
  if (depth_image.size() != point_cloud->cols()) {
    point_cloud->resize(3, depth_image.size());
  }

  const int height = depth_image.height();
  const int width = depth_image.width();
  const float cx = camera_info.center_x();
  const float cy = camera_info.center_y();
  const float fx_inv = 1.f / camera_info.focal_x();
  const float fy_inv = 1.f / camera_info.focal_y();

  Eigen::Matrix3Xf& pc = *point_cloud;
  pc = Eigen::Matrix3Xf::Constant(3, height * width, InvalidDepth::kTooFar);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float z = depth_image.at(u, v)[0];
      if (z != InvalidDepth::kTooClose &&
          z != InvalidDepth::kTooFar) {
        pc(0, v * width + u) = z * (u - cx) * fx_inv;
        pc(1, v * width + u) = z * (v - cy) * fy_inv;
        pc(2, v * width + u) = z;
      }
    }
  }
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double z_near,
                       double z_far,
                       double fov_y,
                       bool show_window)
    : tree_(tree), frame_(RigidBodyFrame<double>()),
      camera_fixed_(true),
      color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y),
      X_WB_initial_(
          Eigen::Translation3d(position[0], position[1], position[2]) *
          Eigen::Isometry3d(math::rpy2rotmat(orientation))),
      renderer_(new RgbdRenderer(
          (Eigen::Translation3d(position[0], position[1], position[2]) *
           Eigen::Isometry3d(math::rpy2rotmat(orientation))) * X_BC_,
           kImageWidth, kImageHeight,
           z_near, z_far,
           fov_y, show_window)) {
  Init(name);
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double z_near,
                       double z_far,
                       double fov_y,
                       bool show_window)
    : tree_(tree), frame_(frame),
      camera_fixed_(false),
      color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y),
      renderer_(new RgbdRenderer(Eigen::Isometry3d(),
                                 kImageWidth, kImageHeight,
                                 z_near, z_far,
                                 fov_y, show_window)) {
  Init(name);
}

void RgbdCamera::Init(const std::string& name) {
  set_name(name);
  const int kVecNum =
      tree_.get_num_positions() + tree_.get_num_velocities();

  state_input_port_ = &this->DeclareInputPort(systems::kVectorValued, kVecNum);

  ImageRgba8U color_image(kImageWidth, kImageHeight);
  color_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageRgba8U(color_image), &RgbdCamera::OutputColorImage);

  ImageDepth32F depth_image(kImageWidth, kImageHeight);
  depth_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageDepth32F(depth_image), &RgbdCamera::OutputDepthImage);

  ImageLabel16I label_image(kImageWidth, kImageHeight);
  label_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageLabel16I(label_image), &RgbdCamera::OutputLabelImage);

  camera_base_pose_port_ = &this->DeclareVectorOutputPort(
      rendering::PoseVector<double>(), &RgbdCamera::OutputPoseVector);

  // Creates rendering world.
  for (const auto& body : tree_.bodies) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    const int body_id = body->get_body_index();
    for (const auto& visual : body->get_visual_elements()) {
      renderer_->RegisterVisual(visual, body_id);
    }
  }

  renderer_->AddFlatTerrain();
}

const InputPortDescriptor<double>& RgbdCamera::state_input_port() const {
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
  for (const auto& body : tree_.bodies) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    const auto X_WBody = tree_.CalcBodyPoseInWorldFrame(cache, *body);

    for (size_t i = 0; i < body->get_visual_elements().size(); ++i) {
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

RgbdCameraDiscrete::RgbdCameraDiscrete(std::unique_ptr<RgbdCamera> camera,
                                       double period)
    : camera_(camera.get()), period_(period) {
  constexpr int width = kImageWidth;
  constexpr int height = kImageHeight;

  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(camera));
  input_port_state_ = builder.ExportInput(camera_->state_input_port());

  // Color image.
  const Value<ImageRgba8U> image_color(width, height);
  const auto* const zoh_color =
      builder.AddSystem<ZeroOrderHold>(period_, image_color);
  builder.Connect(camera_->color_image_output_port(),
                  zoh_color->get_input_port());
  output_port_color_image_ = builder.ExportOutput(zoh_color->get_output_port());

  // Depth image.
  const Value<ImageDepth32F> image_depth(width, height);
  const auto* const zoh_depth =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth);
  builder.Connect(camera_->depth_image_output_port(),
                  zoh_depth->get_input_port());
  output_port_depth_image_ = builder.ExportOutput(zoh_depth->get_output_port());

  // Label image.
  const Value<ImageLabel16I> image_label(width, height);
  const auto* const zoh_label =
      builder.AddSystem<ZeroOrderHold>(period_, image_label);
  builder.Connect(camera_->label_image_output_port(),
                  zoh_label->get_input_port());
  output_port_label_image_ = builder.ExportOutput(zoh_label->get_output_port());

  // No need to place a ZOH on pose output.
  output_port_pose_ =
      builder.ExportOutput(camera_->camera_base_pose_output_port());

  builder.BuildInto(this);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
