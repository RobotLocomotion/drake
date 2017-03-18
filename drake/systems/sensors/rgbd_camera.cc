#include "drake/systems/sensors/rgbd_camera.h"

#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageShiftScale.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/math/roll_pitch_yaw_using_quaternion.h"
#include "drake/systems/rendering/pose_vector.h"
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

const int kPortStateInput = 0;
const int kPortColorImage = 0;
const int kPortDepthImage = 1;
const int kPortLabelImage = 2;
const int kPortCameraPose = 3;

const int kColorImageChannel = 4;
const int kDepthImageChannel = 1;
const int kLabelImageChannel = 1;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes and
// background color.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;

// TODO(kunimatsu-tri) Add support for the arbitrary image size and the depth
// ranges.
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels
const float kDepthRangeNear = 0.5;
const float kDepthRangeFar = 5.0;
const double kTerrainSize = 100.;

// For Zbuffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_DEMAND(false);
  }
  return filepath.substr(0, last_dot);
}

// Defines a color based on its three primary additive colors: red, green, and
// blue. Each of these primary additive colors are in the range of [0, 255].
struct Color {
  int r;  // red
  int g;  // green
  int b;  // blue

  bool operator==(const Color& other) const {
    return this->r == other.r && this->g == other.g && this->b == other.b;
  }
};

// Defines a color based on its three primary additive colors: red, green, and
// blue. Each of these primary additive colors are in the range of [0, 1].
struct NormalizedColor {
  double r;  // red
  double g;  // green
  double b;  // blue
};

// Creates and holds a palette of colors for visualizing different objects in a
// scene (the intent is for a different color to be applied to each identified
// object). The colors are chosen so as to be easily distinguishable. In other
// words, the intensities are spaced as widely as possible given the number of
// required colors. Black, white and gray, which has the same value for all the
// three color channels, are not part of this color palette. This color palette
// can hold up to 1536 colors.
class ColorPalette {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorPalette)
  explicit ColorPalette(int num_colors) {
    const int num = std::ceil(num_colors / 6.);
    DRAKE_DEMAND(num < 256);  // The maximum number of uint8_t.

    for (int i = 0; i < num; ++i) {
      // It is possible to have more colors, but we want the colors to be as
      // distinguishable as possible for visualization purpose.  We can add more
      // colors as needed.
      const int intensity = 255 - i * 255 / num;
      DRAKE_ASSERT(intensity > 0);
      color_palette_.push_back(Color{intensity, 0, 0});
      color_palette_.push_back(Color{0, intensity, 0});
      color_palette_.push_back(Color{0, 0, intensity});
      color_palette_.push_back(Color{intensity, intensity, 0});
      color_palette_.push_back(Color{0, intensity, intensity});
      color_palette_.push_back(Color{intensity, 0, intensity});
    }
  }

  const Color& get_color(int index) const {
    DRAKE_DEMAND(index < static_cast<int>(color_palette_.size()));
    return color_palette_[index];
  }

  const NormalizedColor get_normalized_color(int index) const {
    NormalizedColor color = Normalize(get_color(index));
    return color;
  }

  const NormalizedColor get_normalized_sky_color() const {
    return Normalize(kSkyColor);
  }

  const NormalizedColor get_normalized_terrain_color() const {
    return Normalize(kTerrainColor);
  }

  int LookUpId(const Color& color) const {
    auto it = std::find(color_palette_.begin(), color_palette_.end(), color);
    if (it == color_palette_.end()) {
      if (color == kSkyColor) {
        return RgbdCamera::Label::kNoBody;
      } else if (color == kTerrainColor) {
        return RgbdCamera::Label::kFlatTerrain;
      }
      throw std::runtime_error("ID not found");
    }
    return std::distance(color_palette_.begin(), it);
  }

 private:
  static NormalizedColor Normalize(const Color& color) {
    NormalizedColor normalized;
    normalized.r = color.r / 255.;
    normalized.g = color.g / 255.;
    normalized.b = color.b / 255.;
    return normalized;
  }

  // These colors are chosen so as to be easily distinguished from the colors in
  // color_palette_. They are guaranteed to be distinct from color_palette_
  // because none of their intensity elements are identical.
  const Color kTerrainColor{255, 229, 204};
  const Color kSkyColor{204, 229, 255};
  std::vector<Color> color_palette_;
};

}  // namespace


class RgbdCamera::Impl {
 public:
  Impl(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& frame,
       double fov_y, bool show_window, bool fix_camera);

  Impl(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& frame,
       const Eigen::Vector3d& position, const Eigen::Vector3d& orientation,
       double fov_y, bool show_window, bool fix_camera);

  ~Impl() {}

  static float CheckRangeAndConvertToMeters(float z_buffer_value);

  void DoCalcOutput(const BasicVector<double>& input_vector,
                    systems::SystemOutput<double>* output) const;

  const Eigen::Isometry3d& color_camera_optical_pose() const {
    return X_BC_;
  }

  const Eigen::Isometry3d& depth_camera_optical_pose() const {
    return X_BD_;
  }

  const CameraInfo& color_camera_info() const { return color_camera_info_; }

  const CameraInfo& depth_camera_info() const { return depth_camera_info_; }

  const RigidBodyFrame<double>& frame() const { return frame_; }

  const RigidBodyTree<double>& tree() const { return tree_; }

 private:
  void CreateRenderingWorld();

  void UpdateModelPoses(const KinematicsCache<double>& cache,
                        const Eigen::Isometry3d& X_CW) const;

  void UpdateRenderWindow() const;

  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double>& frame_;
  const CameraInfo color_camera_info_;
  const CameraInfo depth_camera_info_;
  const Eigen::Isometry3d X_BC_;
  const Eigen::Isometry3d X_BD_;
  const Eigen::Isometry3d X_WB_initial_;
  const bool kCameraFixed;
  ColorPalette color_palette_;

  std::map<int, vtkSmartPointer<vtkActor>> id_object_pairs_;
  std::map<int, vtkSmartPointer<vtkActor>> id_label_object_pairs_;
  vtkNew<vtkActor> terrain_actor_;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkRenderer> label_renderer_;
  vtkNew<vtkRenderWindow> render_window_;
  vtkNew<vtkRenderWindow> label_render_window_;
  vtkNew<vtkWindowToImageFilter> depth_buffer_;
  vtkNew<vtkWindowToImageFilter> color_buffer_;
  vtkNew<vtkWindowToImageFilter> label_buffer_;
};


RgbdCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double fov_y, bool show_window, bool fix_camera)
    : tree_(tree), frame_(frame),
      color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y),
      // The color sensor's origin (`Co`) is offset by 0.02 m on the Y axis of
      // the RgbdCamera's base coordinate system (`B`).
      // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
      X_BC_(Eigen::Translation3d(0., 0.02, 0.) *
            (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))),
      // The depth sensor's origin (`Do`) is offset by 0.02 m on Y axis of the
      /// RgbdCamera's base coordinate system (`B`).
      // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
      X_BD_(Eigen::Translation3d(0., 0.02, 0.) *
            (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))),
      X_WB_initial_(
          Eigen::Translation3d(position[0], position[1], position[2]) *
          (Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(orientation[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(orientation[2], Eigen::Vector3d::UnitZ()))),
      kCameraFixed(fix_camera), color_palette_(tree.bodies.size()) {
  if (!show_window) {
    render_window_->SetOffScreenRendering(1);
    label_render_window_->SetOffScreenRendering(1);
  }

  CreateRenderingWorld();

  vtkNew<vtkCamera> camera;
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down.
  camera->SetViewAngle(fov_y * 180. / M_PI);
  camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);

  renderer_->SetActiveCamera(camera.GetPointer());
  auto sky_color = color_palette_.get_normalized_sky_color();
  renderer_->SetBackground(sky_color.r,
                           sky_color.g,
                           sky_color.b);
  render_window_->SetSize(color_camera_info_.width(),
                          color_camera_info_.height());
  render_window_->AddRenderer(renderer_.GetPointer());

  label_renderer_->SetActiveCamera(camera.GetPointer());
  label_renderer_->SetBackground(sky_color.r,
                                 sky_color.g,
                                 sky_color.b);

  label_render_window_->SetSize(color_camera_info_.width(),
                                color_camera_info_.height());
  label_render_window_->AddRenderer(label_renderer_.GetPointer());
  label_render_window_->SetMultiSamples(0);

  // TODO(kunimatsu-tri) Reduce the copy-pasted code for color / depth / label.
  color_buffer_->SetInput(render_window_.GetPointer());
  color_buffer_->SetMagnification(1);
  color_buffer_->SetInputBufferTypeToRGBA();
  color_buffer_->ReadFrontBufferOff();
  color_buffer_->Update();

  depth_buffer_->SetInput(render_window_.GetPointer());
  depth_buffer_->SetMagnification(1);
  depth_buffer_->SetInputBufferTypeToZBuffer();
  depth_buffer_->ReadFrontBufferOff();
  depth_buffer_->Update();

  label_buffer_->SetInput(label_render_window_.GetPointer());
  label_buffer_->SetMagnification(1);
  label_buffer_->SetInputBufferTypeToRGBA();
  label_buffer_->ReadFrontBufferOff();
  label_buffer_->Update();
}

RgbdCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double fov_y, bool show_window, bool fix_camera)
    : Impl::Impl(tree, frame, Eigen::Vector3d(0., 0., 0.),
                 Eigen::Vector3d(0., 0., 0.), fov_y, show_window, fix_camera) {}


void RgbdCamera::Impl::CreateRenderingWorld() {
  auto X_CW = (X_WB_initial_ * X_BC_).inverse();

  for (const auto& body : tree_.bodies) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    if (body->get_visual_elements().empty()) {
      continue;
    }

    int const model_id = body->get_model_instance_id();
    // Assuming that a rigid body owns only a visual element.
    if (body->get_visual_elements().size() >= 2) {
      throw std::runtime_error("RigidBody '" + body->get_name() +
                               "' has two or more visuals.");
    }

    const auto& visual = body->get_visual_elements().at(0);
    // Converts visual's pose in the world to the one in the camera coordinate
    // system.
    auto pose = X_CW * visual.getWorldTransform();
    vtkSmartPointer<vtkTransform> vtk_transform =
        VtkUtil::ConvertToVtkTransform(pose);

    vtkNew<vtkActor> actor;
    vtkNew<vtkActor> actor_for_label;
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

        // Since the cylinder in vtkCylinderSource is y-axis aligned, we need to
        // rotate it to be z-axis aligned because that is what Drake uses.
        vtkNew<vtkTransform> transform;
        transform->RotateX(90);
        vtkNew<vtkTransformPolyDataFilter> transform_filter;
        transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
        transform_filter->SetTransform(transform.GetPointer());
        transform_filter->Update();

        mapper->SetInputConnection(transform_filter->GetOutputPort());
        break;
      }
      case DrakeShapes::MESH: {
        const auto mesh_filename = dynamic_cast<const DrakeShapes::Mesh&>(
            geometry).resolved_filename_.c_str();

        // TODO(kunimatsu-tri) Add support for other file formats.
        vtkNew<vtkOBJReader> mesh_reader;
        mesh_reader->SetFileName(mesh_filename);
        mesh_reader->Update();

        // TODO(kunimatsu-tri) Guessing the texture file name is bad.  Instead,
        // get it from somewhere like `DrakeShapes::MeshWithTexture` when it's
        // implemented.
        // TODO(kunimatsu-tri) Add support for other file formats.
        const std::string texture_file(
            RemoveFileExtension(mesh_filename) + ".png");
        std::ifstream file_exist(texture_file);

        if (file_exist) {
          vtkNew<vtkPNGReader> texture_reader;
          texture_reader->SetFileName(texture_file.c_str());
          texture_reader->Update();

          vtkNew<vtkTexture> texture;
          texture->SetInputConnection(texture_reader->GetOutputPort());
          texture->InterpolateOn();
          actor->SetTexture(texture.GetPointer());
        }

        mapper->SetInputConnection(mesh_reader->GetOutputPort());
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

    // Registers actors.
    if (shape_matched) {
      actor->SetMapper(mapper.GetPointer());
      actor->SetUserTransform(vtk_transform);
      id_object_pairs_[model_id] =
          vtkSmartPointer<vtkActor>(actor.GetPointer());
      renderer_->AddActor(actor.GetPointer());

      const auto& color = color_palette_.get_normalized_color(model_id);
      actor_for_label->GetProperty()->SetColor(color.r, color.g, color.b);
      actor_for_label->GetProperty()->LightingOff();
      actor_for_label->SetMapper(mapper.GetPointer());
      actor_for_label->SetUserTransform(vtk_transform);
      id_label_object_pairs_[model_id] =
          vtkSmartPointer<vtkActor>(actor_for_label.GetPointer());
      label_renderer_->AddActor(actor_for_label.GetPointer());
    }
  }

  // Adds a flat terrain.
  vtkSmartPointer<vtkPlaneSource> plane = VtkUtil::CreateSquarePlane(
      kTerrainSize);
  vtkSmartPointer<vtkTransform> transform =
      VtkUtil::ConvertToVtkTransform(X_CW);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());
  auto color = color_palette_.get_normalized_terrain_color();
  terrain_actor_->GetProperty()->SetColor(color.r,
                                          color.g,
                                          color.b);
  terrain_actor_->GetProperty()->LightingOff();
  terrain_actor_->SetUserTransform(transform);
  renderer_->AddActor(terrain_actor_.GetPointer());
  label_renderer_->AddActor(terrain_actor_.GetPointer());
}

void RgbdCamera::Impl::UpdateModelPoses(
    const KinematicsCache<double>& cache,
    const Eigen::Isometry3d& X_CW) const {
  for (const auto& body : tree_.bodies) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    auto const X_CBody = X_CW * tree_.relativeTransform(
        cache, 0, body->get_body_index());
    vtkSmartPointer<vtkTransform> vtk_transform =
        VtkUtil::ConvertToVtkTransform(X_CBody);
    // `id_object_pairs_` and 'id_label_object_pairs_` are modified here.
    // This is OK because 1) we are just copying data to the memory spaces
    // allocated at the construction time and 2) we are not outputting these
    // data to outside the class.
    auto& actor = id_object_pairs_.at(body->get_model_instance_id());
    actor->SetUserTransform(vtk_transform);

    auto& actor_for_label = id_label_object_pairs_.at(
        body->get_model_instance_id());
    // TODO(kunimatsu-tri) Reduce the copy-pasted code for actor and
    // actor_for_label.
    actor_for_label->SetUserTransform(vtk_transform);
  }

  if (!kCameraFixed) {
    // Updates terrain.
    vtkSmartPointer<vtkTransform> vtk_transform =
        VtkUtil::ConvertToVtkTransform(X_CW);
    // `terrain_actor_` is modified here, but this is OK.  For the detail, see
    // the comment above for `id_object_pairs_`.
    terrain_actor_->SetUserTransform(vtk_transform);
  }
}

void RgbdCamera::Impl::UpdateRenderWindow() const {
  render_window_->Render();
  // TODO(kunimatsu-tri) Reduce the copy-pasted code for color / depth / label.
  color_buffer_->Modified();
  color_buffer_->Update();
  depth_buffer_->Modified();
  depth_buffer_->Update();
  label_buffer_->Modified();
  label_buffer_->Update();
}

void RgbdCamera::Impl::DoCalcOutput(
    const BasicVector<double>& input_vector,
    systems::SystemOutput<double>* output) const {
  const Eigen::VectorXd q = input_vector.CopyToVector().head(
      tree_.get_num_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

  Eigen::Isometry3d X_WB;
  if (kCameraFixed) {
    X_WB = X_WB_initial_;
  } else {
    // Updates camera pose.
    X_WB = tree_.CalcFramePoseInWorldFrame(cache, frame_);
  }

  rendering::PoseVector<double>* const camera_base_pose =
      dynamic_cast<rendering::PoseVector<double>*>(
          output->GetMutableVectorData(kPortCameraPose));

  Eigen::Translation<double, 3> trans = Eigen::Translation<double, 3>(
      X_WB.translation());
  camera_base_pose->set_translation(trans);
  Eigen::Quaterniond quat = Eigen::Quaterniond(X_WB.linear());
  camera_base_pose->set_rotation(quat);

  UpdateModelPoses(cache, (X_WB * X_BC_).inverse());

  UpdateRenderWindow();

  // Outputs the image data.
  sensors::Image<uint8_t>& image =
      output->GetMutableData(kPortColorImage)->GetMutableValue<
        sensors::Image<uint8_t>>();

  sensors::Image<float>& depth_image =
      output->GetMutableData(kPortDepthImage)->GetMutableValue<
        sensors::Image<float>>();

  sensors::Image<int16_t>& label_image =
      output->GetMutableData(kPortLabelImage)->GetMutableValue<
        sensors::Image<int16_t>>();

  const int height = color_camera_info_.height();
  const int width = color_camera_info_.width();
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const int height_reversed = height - v - 1;  // Makes image upside down.

      // We cast `void*` to `uint8_t*` for RGBA, and to `float*` for ZBuffer,
      // respectively. This is because these are the types for pixels internally
      // used in `vtkWindowToImageFiler` class. For more detail, refer to:
      // http://www.vtk.org/doc/release/5.8/html/a02326.html.
      // Converts RGBA to BGRA.
      void* color_ptr = color_buffer_->GetOutput()->GetScalarPointer(u, v, 0);
      image.at(u, height_reversed)[0] = *(static_cast<uint8_t*>(color_ptr) + 2);
      image.at(u, height_reversed)[1] = *(static_cast<uint8_t*>(color_ptr) + 1);
      image.at(u, height_reversed)[2] = *(static_cast<uint8_t*>(color_ptr) + 0);
      image.at(u, height_reversed)[3] = *(static_cast<uint8_t*>(color_ptr) + 3);

      // Updates the depth image.
      const float z_buffer_value = *static_cast<float*>(
          depth_buffer_->GetOutput()->GetScalarPointer(u, v, 0));
      depth_image.at(u, height_reversed)[0] =
          CheckRangeAndConvertToMeters(z_buffer_value);

      // Updates the Label image.
      void* label_ptr = label_buffer_->GetOutput()->GetScalarPointer(u, v, 0);
      Color color{*(static_cast<uint8_t*>(label_ptr) + 0),  // R
                  *(static_cast<uint8_t*>(label_ptr) + 1),  // G
                  *(static_cast<uint8_t*>(label_ptr) + 2)};  // B

      label_image.at(u, height_reversed)[0] =
          static_cast<int16_t>(color_palette_.LookUpId(color));
    }
  }
}

float RgbdCamera::Impl::CheckRangeAndConvertToMeters(float z_buffer_value) {
  float checked_depth;
  // When the depth is either closer than kClippingPlaneNear or further than
  // kClippingPlaneFar, `z_buffer_value` becomes `1.f`.
  if (z_buffer_value == 1.f) {
    checked_depth = InvalidDepth::kError;
  } else {
    // TODO(kunimatsu-tri) Calculate this in a vertex shader.
    float depth = static_cast<float>(kB / (z_buffer_value - kA));

    if (depth > kDepthRangeFar) {
      checked_depth = InvalidDepth::kTooFar;
    } else if (depth < kDepthRangeNear) {
      checked_depth = InvalidDepth::kTooClose;
    } else {
      checked_depth = depth;
    }
  }

  return checked_depth;
}


RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double fov_y,
                       bool show_window)
    : impl_(new RgbdCamera::Impl(tree, RigidBodyFrame<double>(), position,
                                 orientation, fov_y, show_window, true)) {
  Init(name);
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double fov_y,
                       bool show_window)
    : impl_(new RgbdCamera::Impl(tree, frame, fov_y, show_window, false)) {
  Init(name);
}

void RgbdCamera::Init(const std::string& name) {
  set_name(name);
  const int kVecNum =
      impl_->tree().get_num_positions() + impl_->tree().get_num_velocities();
  this->DeclareInputPort(systems::kVectorValued, kVecNum);

  Image<uint8_t> color_image(kImageWidth, kImageHeight, kColorImageChannel);
  this->DeclareAbstractOutputPort(systems::Value<sensors::Image<uint8_t>>(
      color_image));

  Image<float> depth_image(kImageWidth, kImageHeight, kDepthImageChannel);
  this->DeclareAbstractOutputPort(systems::Value<sensors::Image<float>>(
      depth_image));

  Image<int16_t> label_image(kImageWidth, kImageHeight, kLabelImageChannel);
  this->DeclareAbstractOutputPort(systems::Value<sensors::Image<int16_t>>(
      label_image));

  this->DeclareVectorOutputPort(rendering::PoseVector<double>());
}

RgbdCamera::~RgbdCamera() {}

const CameraInfo& RgbdCamera::color_camera_info() const {
  return impl_->color_camera_info();
}

const CameraInfo& RgbdCamera::depth_camera_info() const {
  return impl_->depth_camera_info();
}

const Eigen::Isometry3d& RgbdCamera::color_camera_optical_pose() const {
  return impl_->color_camera_optical_pose();
}

const Eigen::Isometry3d& RgbdCamera::depth_camera_optical_pose() const {
  return impl_->depth_camera_optical_pose();
}

const RigidBodyFrame<double>& RgbdCamera::frame() const {
  return impl_->frame();
}

const RigidBodyTree<double>& RgbdCamera::tree() const {
  return impl_->tree();
}

const InputPortDescriptor<double>& RgbdCamera::state_input_port() const {
  return System<double>::get_input_port(kPortStateInput);
}

const OutputPortDescriptor<double>&
RgbdCamera::color_image_output_port() const {
  return System<double>::get_output_port(kPortColorImage);
}

const OutputPortDescriptor<double>&
RgbdCamera::depth_image_output_port() const {
  return System<double>::get_output_port(kPortDepthImage);
}

const OutputPortDescriptor<double>&
RgbdCamera::label_image_output_port() const {
  return System<double>::get_output_port(kPortLabelImage);
}

const OutputPortDescriptor<double>&
RgbdCamera::camera_base_pose_output_port() const {
  return System<double>::get_output_port(kPortCameraPose);
}

void RgbdCamera::DoCalcOutput(const systems::Context<double>& context,
                              systems::SystemOutput<double>* output) const {
  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, kPortStateInput);

  impl_->DoCalcOutput(*input_vector, output);
}

constexpr float RgbdCamera::InvalidDepth::kError;
constexpr float RgbdCamera::InvalidDepth::kTooFar;
constexpr float RgbdCamera::InvalidDepth::kTooClose;

constexpr int16_t RgbdCamera::Label::kNoBody;
constexpr int16_t RgbdCamera::Label::kFlatTerrain;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
