import lcm
import lcmt_scope_data
from numpy import *
from pylab import *
import time

def fieldChanged(scope_var,data):
    return (data.resetOnXval and data.xdata < scope_var.x[-1]) or (data.scope_id != scope_var.scope_id) or (data.num_points != length(scope_var.x)) or not (data.linespec == scope_var.linespec)

def myHandler(channel, data):
    global scope_data

    varname = channel[channel.find('_scope_')+7:]
    breset = (not scope_data.has_key(varname))
    if fieldChanged(scope_data[varname],data):
        breset = True
        # delete this data handle

    if breset:
        num_points = data.num_points;
        d = 

    msg = lcmt_scope_data.lcmt_scope_data.decode(data)
#    print("Received message on channel \"%s\"" % channel)
    xdata = xdata+[msg.xdata]
    ydata = ydata+[msg.ydata]
    if (len(xdata)>100): 
        xdata=xdata[-100:]
        ydata=ydata[-100:]
    pylab.plot(xdata,ydata,'b')
    pylab.axis('tight')
    t=time.time()
    if (t - drawtime>.1):
      pylab.draw()
      drawtime=t;

lc = lcm.LCM()
subscription = lc.subscribe(".*_scope_.*", myHandler)

scope_data=dict()
max_scope_id = 1
scope_rows = 1
scope_cols = 1

#drawtime=time.time()

pylab.hold(False)


try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)

