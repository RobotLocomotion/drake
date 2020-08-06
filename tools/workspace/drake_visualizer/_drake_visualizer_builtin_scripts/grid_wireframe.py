"""
Sets the grid to be a wireframe rather than the deafult ("Surface with edges").

For motivation, please see:
https://github.com/RobotLocomotion/drake/issues/13828
"""

import director.objectmodel as om


def activate():
    grid = om.findObjectByName("grid")
    grid.setProperty("Surface Mode", "Wireframe")


# Activate the plugin if this script is run directly.
if __name__ == "__main__":
    activate()
