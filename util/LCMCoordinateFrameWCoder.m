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
      obj.channel = name;

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

    function setDefaultChannel(obj,channel)
      typecheck(channel,'char');
      obj.channel = channel;
    end

    function channel = defaultChannel(obj)
      channel = obj.channel;
    end

    function setupLCMInputs(obj,mdl,subsys,subsys_portnum)
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      assignin('base',[mdl,'_subscriber',uid],obj);
      add_block('drake/lcmInput',[mdl,'/in',uid],'channel',['''',obj.channel,''''],'dim',num2str(obj.dim),'lcm_subscriber',[mdl,'_subscriber',uid]);
      add_line(mdl,['in',uid,'/1'],[subsys,'/',num2str(subsys_portnum)]);
    end
    
    function setupLCMOutputs(obj,mdl,subsys,subsys_portnum)
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      assignin('base',[mdl,'_publisher',uid],obj);
      add_block('drake/lcmOutput',[mdl,'/out',uid],'channel',['''',obj.channel,''''],'dim',num2str(obj.dim),'lcm_publisher',[mdl,'_publisher',uid]);
      add_line(mdl,[subsys,'/',num2str(subsys_portnum)],['out',uid,'/1']);
    end
  end
  
  properties
    lcmcoder;
    monitor;
    channel;
  end
end