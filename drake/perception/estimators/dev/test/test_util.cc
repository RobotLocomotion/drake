#include "drake/perception/estimators/dev/test/test_util.h"

using std::string;
using std::vector;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;

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

void IcpVisualizer::PublishCloud(const Matrix3Xd& points,
                                 const string& suffix) {
  bot_core::pointcloud_t pt_msg{};
  PointCloudToLcm(points, &pt_msg);
  vector<uint8_t> bytes(pt_msg.getEncodedSize());
  pt_msg.encode(bytes.data(), 0, bytes.size());
  lcm_.Publish("DRAKE_POINTCLOUD_" + suffix, bytes.data(), bytes.size());
}

}  // namespace estimators
}  // namespace perception
}  // namespace drake
