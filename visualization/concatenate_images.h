#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace visualization {

/** %ConcatenateImages stacks multiple input images into a single output image.

@system
name: ConcatenateImages
input_ports:
- color_image_r0_c0
- color_image_r0_c1
- ...
output_ports:
- color_image
@endsystem

All inputs must be of type ImageRgba8U.

Any input port may be disconnected, in which case it will be interpreted as
zero-sized image.

In case of non-uniform image sizes, any gaps between images will be filled with
all-zero pixels (i.e., with 0% alpha).

@tparam_double_only
@ingroup visualization */
template <typename T>
class ConcatenateImages final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConcatenateImages);

  /** Constructs a %ConcatenateImages system.
  @param rows Number of images to stack vertically.
  @param cols Number of images to stack horizontally. */
  explicit ConcatenateImages(int rows = 1, int cols = 1);

  ~ConcatenateImages() final;

  /** Returns the InputPort for the given (row, col) image. Rows and columns are
  0-indexed, i.e., we have ``0 <= row < rows`` and ``0 <= col < cols``. */
  const systems::InputPort<T>& get_input_port(int row, int col) const;

 private:
  void CalcOutput(const systems::Context<T>& context,
                  systems::sensors::ImageRgba8U* output) const;

  const int rows_;
  const int cols_;
  MatrixX<const systems::InputPort<T>*> inputs_;
};

}  // namespace visualization
}  // namespace drake
