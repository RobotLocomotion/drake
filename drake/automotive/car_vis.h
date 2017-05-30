#pragma once

#include <string>
#include <vector>

#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace automotive {

/// CarVis is a base class that provides visualization geometries and their
/// poses.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
template <typename T>
class CarVis {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CarVis)

  /// The constructor.
  ///
  /// @param id The ID of the vehicle being visualized. This must be unique per
  /// vehicle in the same simulation.
  ///
  /// @param name The name of the vehicle being visualized. This can be any
  /// user-defined value.
  ///
  CarVis(int id, const std::string& name) : id_(id), name_(name) {}

  virtual ~CarVis() {}

  /// Returns the visualization elements.
  virtual const std::vector<lcmt_viewer_link_data>& GetVisElements() const = 0;

  /// Computes and returns the poses of the bodies that constitute the vehicle's
  /// visualization. The provided `X_WM` is the pose of the vehicle model in the
  /// world frame. The origin of the model's frame is assumed to be in the
  /// middle of the vehicle's rear axle. The poses in the returned PoseBundle
  /// are for the visualization's elements, and are also in the world frame. The
  /// size of this bundle is the value returned by num_poses().
  virtual systems::rendering::PoseBundle<T> CalcPoses(
      const Isometry3<T>& X_WM) const = 0;

  /// Returns the ID that was supplied to the constructor.
  int id() const { return id_; }

  /// Returns the name that was supplied to the constructor.
  const std::string& name() const { return name_; }

  /// Returns the number of visualization geometry poses.
  int num_poses() const;

 private:
  const int id_;
  const std::string name_;
};

}  // namespace automotive
}  // namespace drake
