#include "drake/perception/estimators/dev/test/test_util.h"

#include <vtkCleanPolyData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_geometry_compare.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/roll_pitch_yaw.h"

using std::pair;
using std::string;
using std::vector;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Isometry3d;
using Eigen::Quaternion;

using ::testing::AssertionFailure;
using ::testing::AssertionResult;
using ::testing::AssertionSuccess;

namespace drake {
namespace perception {
namespace estimators {

void PointCloudToLcm(const Eigen::Matrix3Xd& pts_W,
                     bot_core::pointcloud_t* pmessage) {
  bot_core::pointcloud_t& message = *pmessage;
  message.points.clear();
  // TODO(eric.cousineau): Better way to get world name that is not RBT
  // specific?
  message.frame_id = "world";
  message.n_channels = 0;
  message.n_points = pts_W.cols();
  message.points.resize(message.n_points);
  for (int i = 0; i < message.n_points; ++i) {
    Eigen::Vector3f pt_W = pts_W.col(i).cast<float>();
    message.points[i] = {pt_W(0), pt_W(1), pt_W(2)};
  }
  message.n_points = message.points.size();
}

Matrix2Xd Generate2DPlane(double space, Interval x, Interval y) {
  // Ensure that we cover the edge, such that we have a balanced centroid for
  // PCA / SVD.
  const int nc = floor(x.width() / space) + 1;
  const int nr = floor(y.width() / space) + 1;
  int i = 0;
  Matrix2Xd out(2, nc * nr);
  for (int c = 0; c < nc; c++) {
    for (int r = 0; r < nr; r++) {
      out.col(i) << c * space + x.min, r * space + y.min;
      i++;
    }
  }
  out.conservativeResize(Eigen::NoChange, i);
  return out;
}

Matrix3Xd Generate2DPlane(double space, PlaneIndices is) {
  // Generate single plane.
  Matrix2Xd p2d = Generate2DPlane(space, is.a.interval, is.b.interval);
  // Map to 3d for upper and lower bound
  const int n = p2d.cols();
  Matrix3Xd p3du(3, 2 * n);
  auto map_into = [&](auto&& xpr, double value) {
    xpr.row(is.a.index) = p2d.row(0);
    xpr.row(is.b.index) = p2d.row(1);
    xpr.row(is.d.index).setConstant(value);
  };
  map_into(p3du.leftCols(n), is.d.interval.min);
  map_into(p3du.rightCols(n), is.d.interval.max);
  return p3du;
}

Matrix3Xd GenerateBoxPointCloud(double space, Bounds box) {
  IntervalIndex ix = {0, box.x};
  IntervalIndex iy = {1, box.y};
  IntervalIndex iz = {2, box.z};
  // Generate for each face
  auto xy_z = Generate2DPlane(space, {ix, iy, iz});
  auto yz_x = Generate2DPlane(space, {iy, iz, ix});
  auto xz_y = Generate2DPlane(space, {ix, iz, iy});
  Matrix3Xd pts(3, xy_z.cols() + yz_x.cols() + xz_y.cols());
  pts << xy_z, yz_x, xz_y;
  return pts;
}

AssertionResult CompareRotMatWithoutAxisSign(
    const Matrix3d& R_expected, const Matrix3d& R_actual, double tolerance) {
  // First, ensure both are rotation matrices.
  AssertionResult check_R_expected =
      ExpectRotMat(R_expected, tolerance);
  if (!check_R_expected) {
    return check_R_expected << "(R_expected)";
  }
  AssertionResult check_R_actual =
      ExpectRotMat(R_actual, tolerance);
  if (!check_R_actual) {
    return check_R_actual << "(R_expected)";
  }
  // Next, check the trace of the absolute expected identity.
  const Matrix3d I_check = R_expected.transpose() * R_actual;
  const double tr = I_check.diagonal().cwiseAbs().sum();
  if (fabs(tr - 3) < tolerance) {
    return AssertionSuccess();
  } else {
    return AssertionFailure()
           << "tr(abs(R_expected' * R_actual)) = " << tr << " != 3 by an error"
           << " of " << fabs(tr - 3) << "\n"
           << "R_expected' * R_actual =\n"
           << I_check;
  }
}

AssertionResult CompareTransformWithoutAxisSign(
    const Isometry3d& X_expected, const Isometry3d& X_actual,
    double tolerance) {
  // Check translation.
  AssertionResult check_translation = CompareMatrices(
      X_expected.translation(), X_actual.translation(), tolerance);
  if (!check_translation) {
    return check_translation;
  }
  // Next, check rotation matrices.
  return CompareRotMatWithoutAxisSign(X_expected.linear(),
                                      X_actual.linear(), tolerance);
}

namespace {

void LoadVTPPointCloud(const std::string& filename, Matrix3Xd *pts,
double distance_tolerance = 0.0) {
  auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();

  // TODO(eric.cousineau): Consider using GetVoidPointer() and doing memory
  // mapping, akin to what the vtk.utils.numpy_support module does.
  vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();
  DRAKE_DEMAND(distance_tolerance >= 0);
  if (distance_tolerance > 0) {
    // Downsample based on distance tolerance.
    const int num_points_full = polyData->GetNumberOfPoints();
    auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    cleaner->SetTolerance(distance_tolerance);
    cleaner->SetInputData(polyData);
    cleaner->Update();
    polyData = cleaner->GetOutput();
    drake::log()->debug("Loading: {}. Downsample from {} to {}",
                        filename, num_points_full,
                        polyData->GetNumberOfPoints());
  }

  const int num_points = polyData->GetNumberOfPoints();
  pts->resize(Eigen::NoChange, num_points);
  Vector3d tmp;
  for (int i = 0; i < num_points; ++i) {
    polyData->GetPoint(i, tmp.data());
    pts->col(i) = tmp;
  }
}

}  // namespace

void GetObjectTestSetup(ObjectTestType type, ObjectTestSetup *setup) {
  switch (type) {
    case kSimpleCuboid: {
      // Simple URDF definition.
      setup->urdf_file =
          "drake/perception/estimators/dev/test/simple_cuboid.urdf";
      // Use a bound whose principal axes (in order) are NOT (x, y, z), but
      // rather (z, x, y).
      const Bounds box(
          Interval(-0.05, 0.05),
          Interval(-0.01, 0.01),
          Interval(-0.2, 0.2));
      const double spacing = 0.01;
      // Generate points in body frame.
      setup->points_B = GenerateBoxPointCloud(spacing, box);
      // Define a generic pose in the world.
      const Vector3d xyz(0.1, 0.2, 0.3);
      const Vector3d rpy(kPi / 3, kPi / 11, kPi / 12);
      setup->X_WB.setIdentity();
      setup->X_WB.linear() << drake::math::rpy2rotmat(rpy);
      setup->X_WB.translation() << xyz;
      break;
    }
    case kBlueFunnelScan: {
      setup->urdf_file =
          "drake/perception/estimators/dev/test/blue_funnel.urdf";
      // `Wm` is the measured transform, from the CORL scene in Director.
      // This is distinguished from `W` so that we may change this in the
      // test code.
      //   f = om.findObjectByName("blue_funnel frame")
      //   print(f.transform)
      Isometry3d X_WmB;
      X_WmB.setIdentity();
      Matrix3d R_WmB;
      R_WmB <<
         0.147663,  0.642632, -0.751811,
         0.988952, -0.105960,  0.103667,
        -0.013042, -0.758812, -0.651179;
      // R_WmB does not precisely belong to O(3) given that its entries are
      // only specified with 6 digits of precision. Therefore we project it
      // onto O(3) before using it:
      X_WmB.linear() = math::ProjectMatToOrthonormalMat(R_WmB);
      X_WmB.translation() << -0.026111, 0.0496843, 0.548844;

      setup->X_WB = X_WmB;
      // Add translation along the x-axis (to simplify debugging with
      // drake-visualizer).
      setup->X_WB.translation() += Vector3d(0.5, 0, 0);

      // Measured points are in the world frame.
      const double distance_tolerance = 0.05;
      Matrix3Xd points_Wm;
      LoadVTPPointCloud(
          FindResourceOrThrow(
              "drake/perception/estimators/dev/test/blue_funnel_meas.vtp"),
          &points_Wm, distance_tolerance);
      setup->points_B = X_WmB.inverse() * points_Wm;
      break;
    }
    default: {
      DRAKE_DEMAND(false);
    }
  }
}

void PointCloudVisualizer::PublishCloud(const Matrix3Xd& points,
                                 const string& suffix) {
  bot_core::pointcloud_t pt_msg{};
  PointCloudToLcm(points, &pt_msg);
  vector<uint8_t> bytes(pt_msg.getEncodedSize());
  pt_msg.encode(bytes.data(), 0, bytes.size());
  lcm_.Publish("DRAKE_POINTCLOUD_" + suffix, bytes.data(), bytes.size());
}

void PointCloudVisualizer::PublishFrames(
    const vector<pair<string, Isometry3d>>& frames) {
  drake::lcmt_viewer_draw msg{};
  const int num_frames = frames.size();
  msg.num_links = num_frames;
  msg.robot_num.resize(num_frames, 0);
  const vector<float> pos = {0, 0, 0};
  const vector<float> quaternion = {1, 0, 0, 0};
  msg.position.resize(num_frames, pos);
  msg.quaternion.resize(num_frames, quaternion);
  for (int i = 0; i < num_frames; ++i) {
    const string& name = frames[i].first;
    const auto& frame = frames[i].second;
    msg.link_name.push_back(name);
    for (int j = 0; j < 3; ++j) {
      msg.position[i][j] = static_cast<float>(frame.translation()[j]);
    }
    Quaternion<float> quat(frame.linear().cast<float>());
    msg.quaternion[i][0] = quat.w();
    msg.quaternion[i][1] = quat.x();
    msg.quaternion[i][2] = quat.y();
    msg.quaternion[i][3] = quat.z();
  }
  vector<uint8_t> bytes(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, bytes.size());
  lcm_.Publish("DRAKE_DRAW_FRAMES", bytes.data(), bytes.size());
}

}  // namespace estimators
}  // namespace perception
}  // namespace drake
