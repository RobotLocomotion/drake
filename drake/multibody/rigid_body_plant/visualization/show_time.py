from director import lcmUtils
import bot_core as lcmbotcore
import time

def onViewerDraw(msg, last_time = [0, 0]):
    t = msg.timestamp * 1e-3 # convert from milliseconds

    rt0 = last_time[0]
    rt1 = time.time()

    t0 = last_time[1]

    dt = t - t0
    drt = rt1 - rt0

    rt_ratio = dt / drt

    last_time[0] = rt1
    last_time[1] = t

    my_text = 'sim time: %.3f, real time factor: %.3f' % (t, rt_ratio)
    vis.updateText(my_text, '')

lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmbotcore.viewer_draw_t, onViewerDraw)
