classdef LCMCoordinateFrame < CoordinateFrame & LCMSubscriber & LCMPublisher
  
  methods
    function obj=LCMCoordinateFrame(name,lcmcoder_or_lcmtype,prefix)
      typecheck(name,'char');
      typecheck(prefix,'char');
      sizecheck(prefix,1);
      
      if isa(lcmcoder_or_lcmtype,'LCMCoder')
        lcmcoder = lcmcoder_or_lcmtype;
      else
        lcmcoder = LCMCoderFromType(lcmcoder_or_lcmtype);
      end
      
      obj = obj@CoordinateFrame(name,lcmcoder.dim(),prefix);
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.channel = name;
      setLCMCoder(obj,lcmcoder);
      setCoordinateNames(obj,lcmcoder.coordinateNames());
    end
  
    function setLCMCoder(obj,lcmcoder)
      typecheck(lcmcoder,'LCMCoder');
      obj.lcmcoder = lcmcoder;
      msg = obj.lcmcoder.encode(0,zeros(obj.dim,1));
      obj.monitor = drake.util.MessageMonitor(msg,obj.lcmcoder.timestampName());
    end
    
    function subscribe(obj,channel)
      assert(~isempty(obj.lcmcoder),'You must set the lcm coder first');
      chash = java.lang.String(channel).hashCode();
      if ~any(chash==obj.subscriptions)  % don't subscribe multiple times to the same channel
        obj.lc.subscribe(channel,obj.monitor);
        obj.subscriptions(end+1)=chash;
      end
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

    function setupLCMInputs(obj,mdl,subsys,subsys_portnum,options)
      if nargin<5, options=struct(); end
      if ~isfield(options,'input_sample_time') options.input_sample_time = .005; end
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      assignin('base',[mdl,'_subscriber',uid],obj);
      add_block('drake/lcmInput',[mdl,'/in',uid],'channel',['''',obj.channel,''''],'dim',num2str(obj.dim),'lcm_subscriber',[mdl,'_subscriber',uid],'sample_time',mat2str(options.input_sample_time));
      add_line(mdl,['in',uid,'/1'],[subsys,'/',num2str(subsys_portnum)]);
    end
    
    function setupLCMOutputs(obj,mdl,subsys,subsys_portnum,options)
      if nargin<5, options=struct(); end
      if ~isfield(options,'output_sample_time') options.output_sample_time = .005; end
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      assignin('base',[mdl,'_publisher',uid],obj);
      add_block('drake/lcmOutput',[mdl,'/out',uid],'channel',['''',obj.channel,''''],'dim',num2str(obj.dim),'lcm_publisher',[mdl,'_publisher',uid],'sample_time',mat2str(options.output_sample_time));
      add_line(mdl,[subsys,'/',num2str(subsys_portnum)],['out',uid,'/1']);
    end
  end
  
  properties
    lc;
    lcmcoder=[];
    monitor=[];
    channel;
    subscriptions;
  end
end