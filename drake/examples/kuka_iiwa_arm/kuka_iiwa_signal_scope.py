import numpy
import colorsys



addPlot(timeWindow=15, yLimits=[-3.14, 3.14])
addSignals('IIWA_STATUS', msg.utime, msg.joint_position_measured, range(7))
addSignals('IIWA_STATUS', msg.utime, msg.joint_position_commanded, range(7))


addPlot(timeWindow=15, yLimits=[-3.14, 3.14])
addSignals('IIWA_COMMAND', msg.utime, msg.joint_position, range(7))

