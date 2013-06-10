classdef LCMCoordinateFrameWCoder < CoordinateFrame & LCMSubscriber & LCMPublisher
  
  methods
    function obj=LCMCoordinateFrameWCoder(name,dim,prefix,lcmcoder)
      typecheck(name,'char');
      typecheck(dim,'double');
      sizecheck(dim,1);
      typecheck(prefix,'char');
      sizecheck(prefix,1);
      
      obj = obj@CoordinateFrame(name,dim,prefix);
      if (nargin>3)
        obj = setLCMCoder(obj,lcmcoder);
      end
      obj.channel = name;
    end
  
    function obj = setLCMCoder(obj,lcmcoder)
      typecheck(lcmcoder,'LCMCoder');
      obj.lcmcoder = lcmcoder;
      msg = obj.lcmcoder.encode(0,zeros(obj.dim,1));
      obj.monitor = drake.util.MessageMonitor(msg,obj.lcmcoder.timestampName());
      obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
    function obj = subscribe(obj,channel)
      obj.lc.subscribe(channel,obj.monitor);
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
    
    function [x,t] = getMessage(obj)
      data = obj.monitor.getMessage();
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
    
    function t = getLastTimestamp(obj)
      t = obj.monitor.getLastTimestamp();
    end
    
    function publish(obj,t,x,channel,varargin)
      msg = obj.lcmcoder.encode(t,x,varargin{:});
      obj.lc.publish(channel,msg);
    end

    function setDefaultChannel(obj,channel)
      typecheck(channel,'char');
      obj.channel = channel;
    end

    function channel = defaultChannel(obj)
      channel = obj.channel;
    end
    
    function markAsRead(obj)
      obj.monitor.markAsRead();
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
    lc;
    lcmcoder=[];
    monitor;
    channel;
  end
end