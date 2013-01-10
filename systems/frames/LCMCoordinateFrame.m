classdef LCMCoordinateFrame < CoordinateFrame & LCMSubscriber & LCMPublisher & Singleton
  
  methods
    function obj = LCMCoordinateFrame(name,lcmtype,prefix)
      typecheck(name,'char');
      typecheck(prefix,'char');
      sizecheck(prefix,[1 1]);
      if (ischar(lcmtype))
        lcmtype = eval(lcmtype);
      end
      if (~any(strcmp('getClass',methods(lcmtype))))
        error('lcmtype should be a valid java lcm object, or the string describing it');
      end
      lcmtype=lcmtype.getClass();
      
      has_timestamp=false;
      names={};
      f = lcmtype.getFields;
      for i=1:length(f)
        fname = char(f(i).getName());
        if strncmp(fname,'LCM_FINGERPRINT',15), continue; end
        if strcmp(fname,'timestamp'), 
          if ~strcmp(f(i).getGenericType.getName,'long')
            error('by convention, the timestamp field should be type int64_t');
          end
          has_timestamp=true; 
          continue; 
        end
        
        names{end+1}=fname;
      end
      if ~has_timestamp
        error('by convention, all lcm types should have a timestamp field of type int64_t');
      end
      
      obj = obj@CoordinateFrame(name,length(names),prefix);
%      obj = obj@Singleton(char(lcmtype.getName));  
%      commented out because it doesn't make sense to
%      check anything other than the classname here, since I can't return a
%      different class type from this constructor.
      obj.lcmtype = lcmtype;
      obj.channel = name;
      
      constructors = lcmtype.getConstructors();
      for i=1:length(constructors)
        f = constructors(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          obj.lcmtype_constructor = constructors(i);
        end
      end
      if isempty(obj.lcmtype_constructor)
        error('didn''t find a constructor for this lcmtype');
      end
      setCoordinateNames(obj,names);
    end
    
    function obj = subscribe(obj,channel)
      lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');
      obj.monitor = drake.util.MessageMonitor(obj.lcmtype,'timestamp');
      lc.subscribe(channel,obj.monitor);
    end
    
    function [x,t] = getNextMessage(obj,timeout)
      if isempty(obj.monitor)
        error('Drake:LCMCoordinateFrame:NoMonitor','You must subscribe to a channel first'); 
      end
      data = getNextMessage(obj.monitor,timeout);
      if isempty(data)
        x=[];
        t=[];
      else
        msg = obj.lcmtype_constructor.newInstance(data);
        x=zeros(obj.dim,1);
        for i=1:obj.dim
          eval(['x(',num2str(i),') = msg.',CoordinateFrame.stripSpecialChars(obj.coordinates{i}),';']);
        end
        t = msg.timestamp/1000;
        obj.last_x = x;
        obj.last_t = t;
      end
    end
    
    function [x,t] = getCurrentValue(obj)
      [x,t]=getNextMessage(obj,0);
      if isempty(t) 
        if (obj.last_x)
          x = obj.last_x;
          t = obj.last_t;
        else
          x = zeros(obj.dim,1);
          t = 0;
        end
      end
    end
    
    function publish(obj,t,x,channel)
      sizecheck(t,1);
      sizecheck(x,[obj.dim,1]);
      msg = obj.lcmtype.newInstance();
      msg.timestamp = t*1000;
      for i=1:obj.dim
        eval(['msg.',CoordinateFrame.stripSpecialChars(obj.coordinates{i}),' = x(',num2str(i),');']);
      end
      lc = lcm.lcm.LCM.getSingleton();
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
    
  end
  
  properties
    lcmtype
    monitor=[];
    last_x;
    last_t;
    lcmtype_constructor;
    channel;
  end
  
end
