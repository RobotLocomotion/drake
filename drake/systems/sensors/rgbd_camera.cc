#include "drake/systems/sensors/rgbd_camera.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageShiftScale.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPNGReader.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkWindowToImageFilter.h>
#include <Eigen/Dense>

#include "drake/math/roll_pitch_yaw_using_quaternion.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"

// TODO(kunimatsu-tri) Refactor RenderingWorld out from RgbdCamera,
// so that other vtk dependent sensor simulators can share the RenderingWorld
// without duplicating it.

namespace drake {
namespace systems {
namespace sensors {
namespace {

const int kColorImageChannel = 4;
const int kDepthImageChannel = 1;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes and
// background color.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kBackgoundColor[3] = {0.8, 0.898, 1.};

// TODO(kunimatsu-tri) Add support for the arbitrary image size and the depth
// ranges.
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels
const double kDepthRangeNear = 0.5;
const double kDepthRangeFar = 5.0;

const double kPlaneSize = 100.;
const double kPlaneColor[3] = {1., 0.898, 0.797};

// For Zbuffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

// TODO(kunimatsu-tri) Calculate this in vertex shader.
double ConvertZbufferToMeters(float z_buffer_value) {
  return kB / (z_buffer_value - kA);
}

std::string RemoveFileExtention(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_ASSERT(false);
  }
  return filepath.substr(0, last_dot);
}

}  // anonymous namespace


class RgbdCamera::Impl {
 public:
  Impl(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& frame,
       double fov_y, bool show_window);

  Impl(const RigidBodyTree<double>& tree, const Eigen::Vector3d& position,
       const Eigen::Vector3d& orientation, double fov_y, bool show_window);

  ~Impl() {}

  const Eigen::Isometry3d& get_base_pose() const { return world_to_base_; }

  const Eigen::Isometry3d& get_color_camera_pose() const {
    return base_to_color_camera_;
  }

  const Eigen::Isometry3d& get_depth_camera_pose() const {
    return base_to_depth_camera_;
  }

  const CameraInfo& get_color_camera_info() const { return color_camera_info_; }

  const CameraInfo& get_depth_camera_info() const { return depth_camera_info_; }

  const RigidBodyTree<double>& get_tree() const { return tree_; }

  void DoCalcOutput(const BasicVector<double>& input_vector,
                    systems::SystemOutput<double>* output) const;

 private:
  void CreateRenderingWorld();

  void UpdateModelPoses(const VectorBase<double>& vector_base) const;

  void UpdateRenderWindow() const;

  const RigidBodyTree<double>& tree_;
  const CameraInfo color_camera_info_;
  const CameraInfo depth_camera_info_;
  Eigen::Isometry3d world_to_base_;
  const Eigen::Isometry3d base_to_color_camera_;
  const Eigen::Isometry3d base_to_depth_camera_;

  std::map<int, vtkSmartPointer<vtkActor>> id_object_pairs_;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkRenderWindow> render_window_;
  vtkNew<vtkWindowToImageFilter> depth_buffer_;
  vtkNew<vtkWindowToImageFilter> color_buffer_;
};


RgbdCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double fov_y,
                       bool show_window)
    : tree_(tree),
      color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y),
      // Color camera's origin is offset by 0.02m on Y axis in the camera base
      // coordinate system.
      // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
      base_to_color_camera_(Eigen::Translation3d(0., 0.02, 0.)),
      // Depth camera's origin is offset by 0.02m on Y axis in the camera base
      // coordinate system.
      // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
      base_to_depth_camera_(Eigen::Translation3d(0., 0.02, 0.)) {
  if (!show_window) {
    render_window_->SetOffScreenRendering(1);
  }

  // Rgbd camera's base pose in the world.
  auto axis_angle = drake::math::rpy2axis(orientation);
  world_to_base_ = Eigen::AngleAxisd(axis_angle[3],
      Eigen::Vector3d(axis_angle[0], axis_angle[1], axis_angle[2]));
  world_to_base_.translation() = Eigen::Vector3d(
      position[0], position[1], position[2]);

  CreateRenderingWorld();

  vtkNew<vtkCamera> camera;
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(1., 0., 0.);
  camera->SetViewUp(0., 0, 1.);
  camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);

  renderer_->SetActiveCamera(camera.GetPointer());
  renderer_->SetBackground(kBackgoundColor[0],
                           kBackgoundColor[1],
                           kBackgoundColor[2]);

  render_window_->SetSize(color_camera_info_.width(),
                          color_camera_info_.height());
  render_window_->AddRenderer(renderer_.GetPointer());

  color_buffer_->SetInput(render_window_.GetPointer());
  color_buffer_->SetMagnification(1);
  color_buffer_->SetInputBufferTypeToRGBA();
  color_buffer_->Update();

  depth_buffer_->SetInput(render_window_.GetPointer());
  depth_buffer_->SetMagnification(1);
  depth_buffer_->SetInputBufferTypeToZBuffer();
  depth_buffer_->Update();
}

RgbdCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double fov_y,
                       bool show_window)
    : tree_(tree), color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y) {
  // TODO(kunimatsu) Implement this
  std::runtime_error("Not implemented");
}

void RgbdCamera::Impl::CreateRenderingWorld() {
  auto camera_to_world =  (world_to_base_ * base_to_color_camera_).inverse();

  for (const auto& body : tree_.bodies) {
    if (body->get_name() == "world") {
      continue;
    }

    int const model_id = body->get_model_instance_id();
    // Assuming that a rigid body owns only a visual element.
    if (body->get_visual_elements().size() >= 2) {
      throw std::runtime_error("RigidBody '" + body->get_name() +
                               "' has more than two visuals.");
    }
    for (const auto& visual : body->get_visual_elements()) {
      // Converts visual's pose in the world to the one in the camera coordinate
      // system.
      auto pose = camera_to_world * visual.getWorldTransform();
      vtkSmartPointer<vtkTransform> vtk_transform =
          VtkUtil::ConvertToVtkTransform(pose);

      vtkNew<vtkActor> actor;
      vtkNew<vtkPolyDataMapper> mapper;
      bool shape_matched = true;
      const DrakeShapes::Geometry& geometry = visual.getGeometry();
      switch (visual.getShape()) {
        case DrakeShapes::BOX: {
          auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
          vtkNew<vtkCubeSource> vtk_cube;
          vtk_cube->SetXLength(box.size(0));
          vtk_cube->SetYLength(box.size(1));
          vtk_cube->SetZLength(box.size(2));

          mapper->SetInputConnection(vtk_cube->GetOutputPort());
          break;
        }
        case DrakeShapes::SPHERE: {
          auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
          vtkNew<vtkSphereSource> vtk_sphere;
          vtk_sphere->SetRadius(sphere.radius);
          vtk_sphere->SetThetaResolution(50);
          vtk_sphere->SetPhiResolution(50);

          mapper->SetInputConnection(vtk_sphere->GetOutputPort());
          break;
        }
        case DrakeShapes::CYLINDER: {
          auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
          vtkNew<vtkCylinderSource> vtk_cylinder;
          vtk_cylinder->SetHeight(cylinder.length);
          vtk_cylinder->SetRadius(cylinder.radius);
          vtk_cylinder->SetResolution(50);

          mapper->SetInputConnection(vtk_cylinder->GetOutputPort());
          break;
        }
        case DrakeShapes::MESH: {
          auto m = dynamic_cast<const DrakeShapes::Mesh&>(geometry);

          // TODO(kunimatsu-tri) Add support for other file formats.
          vtkNew<vtkOBJReader> mesh_reader;
          mesh_reader->SetFileName(m.resolved_filename_.c_str());
          mesh_reader->Update();

          // TODO(kunimatsu-tri) Add support for other file formats.
          vtkNew<vtkPNGReader> texture_reader;
          texture_reader->SetFileName(std::string(RemoveFileExtention(
              m.resolved_filename_.c_str()) + ".png").c_str());
          texture_reader->Update();

          vtkNew<vtkTexture> texture;
          texture->SetInputConnection(texture_reader->GetOutputPort());
          texture->InterpolateOn();

          mapper->SetInputConnection(mesh_reader->GetOutputPort());
          actor->SetTexture(texture.GetPointer());
          break;
        }
        case DrakeShapes::CAPSULE: {
          // TODO(kunimatsu-tri) Implement this as needed.
          shape_matched = false;
          break;
        }
        default: {
          shape_matched = false;
          break;
        }
      }

      // Registers actors
      if (shape_matched) {
        actor->SetMapper(mapper.GetPointer());
        actor->SetUserTransform(vtk_transform);
        id_object_pairs_[model_id] =
            vtkSmartPointer<vtkActor>(actor.GetPointer());
        renderer_->AddActor(actor.GetPointer());
      }
    }
  }

  // Adds a flat terrain.
  // TODO(kunimatsu-tri) This also should be updated at every time step when the
  // camera pose update capability is implemented.
  vtkSmartPointer<vtkPlaneSource> plane = VtkUtil::CreateSquarePlane(
      kPlaneSize);

  vtkSmartPointer<vtkTransform> transform =
      VtkUtil::ConvertToVtkTransform(camera_to_world);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInput(plane->GetOutput());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper.GetPointer());
  actor->GetProperty()->SetColor(kPlaneColor[0],
                                 kPlaneColor[1],
                                 kPlaneColor[2]);


  actor->SetUserTransform(transform);
  renderer_->AddActor(actor.GetPointer());
}


void RgbdCamera::Impl::UpdateModelPoses(
    const VectorBase<double>& vector_base) const {
  const Eigen::VectorXd q = vector_base.CopyToVector().head(
      tree_.get_num_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

  // TODO(kunimatsu-tri) Update camera frame here and the flat terrain too.
  auto camera_to_world =  (world_to_base_ * base_to_color_camera_).inverse();

  for (const auto& body : tree_.bodies) {
    if (body->get_name() == "world") {
      continue;
    }

    auto pose = camera_to_world * tree_.relativeTransform(
        cache, 0, body->get_body_index());

    vtkSmartPointer<vtkTransform> vtk_transform =
        VtkUtil::ConvertToVtkTransform(pose);

    auto& actor = id_object_pairs_.at(body->get_model_instance_id());
    actor->SetUserTransform(vtk_transform);
  }
}

void RgbdCamera::Impl::UpdateRenderWindow() const {
  render_window_->Render();
  color_buffer_->Modified();
  color_buffer_->Update();
  depth_buffer_->Modified();
  depth_buffer_->Update();
}

void RgbdCamera::Impl::DoCalcOutput(
    const BasicVector<double>& input_vector,
    systems::SystemOutput<double>* output) const {
  UpdateModelPoses(input_vector);

  UpdateRenderWindow();

  // Outputs the image data
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  drake::systems::sensors::Image<uint8_t>& image =
      mutable_data->GetMutableValue<
        drake::systems::sensors::Image<uint8_t>>();

  systems::AbstractValue* mutable_data_d = output->GetMutableData(1);
  drake::systems::sensors::Image<float>& depth_image =
      mutable_data_d->GetMutableValue<
        drake::systems::sensors::Image<float>>();

  const auto height = color_camera_info_.height();
  const auto width = color_camera_info_.width();
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const int height_reversed = height - v - 1;  // Makes image upside down

      // Color image
      void* color_ptr = color_buffer_->GetOutput()->GetScalarPointer(u, v, 0);
      // Converts RGBA to BGRA
      image.at(u, height_reversed)[0] = *(static_cast<uint8_t*>(color_ptr) + 2);
      image.at(u, height_reversed)[1] = *(static_cast<uint8_t*>(color_ptr) + 1);
      image.at(u, height_reversed)[2] = *(static_cast<uint8_t*>(color_ptr) + 0);
      image.at(u, height_reversed)[3] = *(static_cast<uint8_t*>(color_ptr) + 3);

      // Depth image
      float depth_buffer_value = *static_cast<float*>(
          depth_buffer_->GetOutput()->GetScalarPointer(u, v, 0));
      // When the depth is either closer than kClippingPlaneNear or further than
      // kClippingPlaneFar, `depth_value` becomes `1.f`.
      if (depth_buffer_value == 1.f) {
        depth_image.at(u, height_reversed)[0] = InvalidDepth::kError;
      } else {
        double depth_value = ConvertZbufferToMeters(depth_buffer_value);
        if (depth_value > kDepthRangeFar) {
          depth_image.at(u, height_reversed)[0] = InvalidDepth::kTooFar;
        } else if (depth_value < kDepthRangeNear) {
          depth_image.at(u, height_reversed)[0] = InvalidDepth::kTooClose;
        } else {
          depth_image.at(u, height_reversed)[0] =
              static_cast<float>(depth_value);
        }
      }
    }
  }
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double frame_rate,
                       double fov_y,
                       bool show_window)
    : impl_(new RgbdCamera::Impl(tree, position, orientation, fov_y,
                                 show_window)),
      frame_interval_(1. / frame_rate) {
  set_name(name);
  const int vec_num =  tree.get_num_positions() + tree.get_num_velocities();
  input_port_index_ = this->DeclareInputPort(
      systems::kVectorValued, vec_num).get_index();

  this->DeclareAbstractOutputPort();
  this->DeclareAbstractOutputPort();
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double frame_rate,
                       double fov_y,
                       bool show_window)
    : impl_(new RgbdCamera::Impl(tree, frame, fov_y, show_window)),
      frame_interval_(1. / frame_rate) {
  set_name(name);
  const int vec_num =  tree.get_num_positions() + tree.get_num_velocities();
  input_port_index_ = this->DeclareInputPort(
      systems::kVectorValued, vec_num).get_index();

  this->DeclareAbstractOutputPort();
  this->DeclareAbstractOutputPort();
}

RgbdCamera::~RgbdCamera() {}

const CameraInfo& RgbdCamera::get_color_camera_info() const {
  return impl_->get_color_camera_info();
}

const CameraInfo& RgbdCamera::get_depth_camera_info() const {
  return impl_->get_depth_camera_info();
}

const Eigen::Isometry3d& RgbdCamera::get_base_pose() const {
  return impl_->get_base_pose();
}

const Eigen::Isometry3d& RgbdCamera::get_color_camera_pose() const {
  return impl_->get_color_camera_pose();
}

const Eigen::Isometry3d& RgbdCamera::get_depth_camera_pose() const {
  return impl_->get_depth_camera_pose();
}

const RigidBodyTree<double>& RgbdCamera::get_tree() const {
  return impl_->get_tree();
}

std::unique_ptr<SystemOutput<double>> RgbdCamera::AllocateOutput(
    const Context<double>& context) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();

  sensors::Image<uint8_t> color_image(kImageWidth, kImageHeight,
                                      kColorImageChannel);
  output->add_port(
      std::make_unique<systems::Value<sensors::Image<uint8_t>>>(color_image));

  sensors::Image<float> depth_image(kImageWidth, kImageHeight,
                                    kDepthImageChannel);
  output->add_port(
      std::make_unique<systems::Value<sensors::Image<float>>>(depth_image));

  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void RgbdCamera::DoCalcOutput(const systems::Context<double>& context,
                              systems::SystemOutput<double>* output) const {
  const double current_time = context.get_time();
  if (current_time == 0. ||
      (current_time - previous_output_time_ < frame_interval_)) {
    return;
  }

  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, input_port_index_);

  impl_->DoCalcOutput(*input_vector, output);

  previous_output_time_ = current_time;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
