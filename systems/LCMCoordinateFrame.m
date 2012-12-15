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
      
      constructors = lcmtype.getDeclaredConstructors;
      for i=1:length(constructors)
        f = constructors(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          obj.decode_constructor = constructors(i);
        end
      end
      if isempty(obj.decode_constructor)
        error('didn''t find decoder constructor');
      end
      obj = setCoordinateNames(obj,names);
    end
    
    function obj = subscribe(obj,channel)
      if ~isempty(obj.aggregator)
        delete obj.aggregator;
      end
      
      lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.aggregator.setMaxMessages(1);  % make it a last-message-only queue
  
      lc.subscribe(options.inchannel,obj.aggregator);
    end
    
    function [x,t] = getNextMessage(obj,timeout)
      if isempty(obj.aggregator)
        error('Drake:LCMCoordinateFrame:NoAggregator','You must subscribe to a channel first'); 
      end
      msg = getNextMessage(obj.aggregator,timeout);
      if (~isempty(msg))
        msg = obj.decode_constructor.newInstance(msg.data);
        x=zeros(obj.dim,1);
        for i=1:obj.dim
          eval(['x(',num2str(i),') = msg.',CoordinateFrame.stripSpecialChars(obj.coordinates{i}),';']);
        end
        t = msg.timestamp/1000;
        obj.last_x = x;
        obj.last_t = t;
        timedout=false;
      else
        x=[];
        t=[];
      end
    end
    
    function [x,t] = getCurrentValue(obj)
      [x,t]=getNextMessage(obj,0);
      if isempty(t)
        x = obj.last_x;
        t = obj.last_t;
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
    end
  end
  
  properties
    lcmtype
    aggregator=[];
    last_x;
    last_t;
  end
  
end
