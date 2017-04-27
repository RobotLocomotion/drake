'''
Usage: This program should be launched using the command line specified in the
       kuka_sim.pmd file.
'''

import time

from director import mainwindowapp
from director import robotsystem
from director import applogic
from director import lcmUtils
from PythonQt import QtGui, QtCore
import drake as lcmdrake


class KukaSimInfoLabel(object):
    '''
    Displays simulation time and frequency in the status bar
    using information from the IIWA_STATUS lcm channel.
    '''

    def __init__(self, statusBar):
        self.label = QtGui.QLabel('')
        statusBar.addPermanentWidget(self.label)

        self.sub = lcmUtils.addSubscriber(
            'IIWA_STATUS', lcmdrake.lcmt_iiwa_status, self.onIiwaStatus)
        self.sub.setSpeedLimit(30)

        self.label.text = '[waiting for sim status]'

    def onIiwaStatus(self, msg):
        simTime = msg.utime * 1e-6
        simFreq = self.sub.getMessageRate()
        self.label.text = 'Sim freq: %d hz  |  Sim time: %.2f' % (simFreq,
                                                                  simTime)


def sendGripperCommand(targetPositionMM, force):
    msg = lcmdrake.lcmt_schunk_wsg_command()
    msg.utime = int(time.time() * 1e6)
    msg.force = force
    msg.target_position_mm = targetPositionMM
    lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)


def gripperOpen():
    sendGripperCommand(100, 40)


def gripperClose():
    sendGripperCommand(15, 40)


def setupToolbar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.app.addToolBarAction(
        toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    app.app.addToolBarAction(
        toolBar, 'Gripper Close', icon='', callback=gripperClose)


# create a default mainwindow app
app = mainwindowapp.construct(globalsDict=globals())

# load a minimal robot system with ik planning
robotSystem = robotsystem.create(app.view, planningOnly=True)

# add the teleop and playback panels to the mainwindow
app.app.addWidgetToDock(robotSystem.teleopPanel.widget,
                        QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget,
                        QtCore.Qt.BottomDockWidgetArea)

setupToolbar()

# show sim time in the status bar
infoLabel = KukaSimInfoLabel(app.mainWindow.statusBar())

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
if ikPlanner.planningMode == 'pydrake':
    ikPlanner.plannerPub._setupLocalServer()

# change the default animation mode of the playback panel
robotSystem.playbackPanel.animateOnExecute = True

# disable pointwise ik by default
ikPlanner.getIkOptions().setProperty('Use pointwise', False)
ikPlanner.getIkOptions().setProperty('Max joint degrees/s', 60)

# initialize the listener for the pose gui
ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)

# set the default camera view
applogic.resetCamera(viewDirection=[-1, 0, 0], view=app.view)

# start!
app.app.start()
