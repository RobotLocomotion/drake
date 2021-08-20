"""Add buttons to drake-visualizer to open and close the Schunk WSG.

Example usage:

    drake-visualizer \
        --script manipulation/schunk_wsg/schunk_buttons.py

This will create buttons labelled 'Gripper Open' and 'Gripper Close'
on the main toolbar.

The 'sendGripperCommand' function can also be called from the python
console to send arbitrary commands.

"""

# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope

from director import lcmUtils

import drake as lcmdrake

print("DRAKE DEPRECATED: This script will be removed from Drake on or after 2021-12-01.")

def sendGripperCommand(targetPositionMM, force):
    msg = lcmdrake.lcmt_schunk_wsg_command()
    msg.utime = int(time.time()*1e6)
    msg.force = force
    msg.target_position_mm = targetPositionMM
    lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)


def gripperOpen():
    gripper_open_position_mm = 100
    gripper_open_force = 40
    sendGripperCommand(gripper_open_position_mm, gripper_open_force)


def gripperClose():
    # Closing to 0mm can smash the fingers together and keep applying
    # force even when no object is grasped.
    gripper_close_position_mm = 8
    gripper_close_force = 40
    sendGripperCommand(gripper_close_position_mm, gripper_close_force)


toolBar = applogic.findToolBar('Main Toolbar')
app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=gripperOpen)
app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=gripperClose)
