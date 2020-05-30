"""
Limits clipping range as mentioned here:
https://github.com/RobotLocomotion/drake/issues/13446#issuecomment-635997871

For background, see:
https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
"""

# Use a range that is generally valid for robot manipulation applications.
# N.B. Depending on the choice of these numbers, it may be necessary to only
# limit one, or both. For simplicity, we will always limit both.
MAX_NEAR = 0.01  # 1cm
MAX_FAR = 1e3  # 1km


def activate(view):

    def on_render(render_window, event):
        near, far = view.camera().GetClippingRange()
        near = min(MAX_NEAR, near)
        far = min(MAX_FAR, far)
        assert near > 0
        assert far > 0
        assert far > near
        view.camera().SetClippingRange(near, far)

    view.renderWindow().AddObserver('StartEvent', on_render)


# Activate the plugin if this script is run directly.
if __name__ == "__main__":
    activate()
