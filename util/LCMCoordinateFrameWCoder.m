classdef LCMCoordinateFrameWCoder < CoordinateFrame & LCMSubscriber & LCMPublisher
  
  methods
    function obj=LCMCoordinateFrameWCoder(name,dim,prefix,lcmcoder)
      typecheck(name,'char');
      typecheck(dim,'double');
      sizecheck(dim,1);
      typecheck(prefix,'char');
      sizecheck(prefix,1);
      typecheck(lcmcoder,'LCMCoder');
      
      obj = obj@CoordinateFrame(name,dim,prefix);
      obj.lcmcoder = lcmcoder;

      msg = obj.lcmcoder.encode(0,zeros(obj.dim,1));
      obj.monitor = drake.util.MessageMonitor(msg,obj.lcmcoder.timestampName());
    end
  
    function obj = subscribe(obj,channel)
      lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');
      lc.subscribe(channel,obj.monitor);
    end
    
    function [x,t] = getNextMessage(obj,timeout)   % x=t=[] if timeout
      data = obj.monitor.getNextMessage(timeout);
      if isempty(data)
        x=[];
        t=[];
      else
        [x,t] = obj.lcmcoder.decode(data);
      end
    end
    
    function [x,t] = getCurrentValue(obj)
      data = obj.monitor.getMessage();
      if isempty(data)
        x=[];
        t=[];
      else
        [x,t] = obj.lcmcoder.decode(data);
      end
    end
    
    function publish(obj,t,x,channel)
      lc = lcm.lcm.LCM.getSingleton();
      msg = obj.lcmcoder.encode(t,x);
      lc.publish(channel,msg);
    end
    
    function str = defaultChannel(obj)
      str = obj.name;
    end
  
  end
  
  properties
    lcmcoder;
    monitor;
  end
end