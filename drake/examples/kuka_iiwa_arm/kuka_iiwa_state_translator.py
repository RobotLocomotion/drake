import lcm
import drake as lcmdrake
import bot_core as lcmbotcore

from director import consoleapp
from director import lcmUtils
from director import robotstate

def onIiwaStatus(msg):
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + list(msg.joint_position_measured)
    stateMsg = robotstate.drakePoseToRobotState(q)
    stateMsg.utime = msg.utime
    lcmUtils.publish('EST_ROBOT_STATE', stateMsg)


subscriber = lcmUtils.addSubscriber('IIWA_STATUS', lcmdrake.lcmt_iiwa_status, onIiwaStatus)
consoleapp.ConsoleApp.start()
