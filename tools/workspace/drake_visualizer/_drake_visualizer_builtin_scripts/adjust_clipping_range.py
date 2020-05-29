"""
Adjusts clipping range per advice mentioned here:
https://github.com/RobotLocomotion/drake/issues/13446
"""

MAX_NEAR = 0.05
MAX_FAR = 1e4


def activate(view):

    def on_render(render_window, event):
        near, far = view.camera().GetClippingRange()
        near = min(MAX_NEAR, near)
        far = min(MAX_FAR, far)
        view.camera().SetClippingRange(near, far)

    view.renderWindow().AddObserver('StartEvent', on_render)


# Activate the plugin if this script is run directly.
if __name__ == "__main__":
    activate()
