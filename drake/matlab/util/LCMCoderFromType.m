classdef LCMCoderFromType < LCMCoder
  
  % note: might be better (faster) to move this to java
  
  methods 
    function obj = LCMCoderFromType(lcmtype)
      checkDependency('lcm');
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
      
      obj.lcmtype = lcmtype;
      obj.coordinate_names = names;
      
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
      
    end
    
    function d = dim(obj)
      d = length(obj.coordinate_names);
    end
    
    function str = timestampName(obj)
      str = 'timestamp';
    end
    
    function str = coordinateNames(obj)
      str = obj.coordinate_names;
    end
    
    function [x,t] = decode(obj,data)
      if isempty(data)
        x=[];
        t=[];
      else
        msg = obj.lcmtype_constructor.newInstance(data);
        x=zeros(obj.dim,1);
        for i=1:obj.dim
          eval(['x(',num2str(i),') = msg.',CoordinateFrame.stripSpecialChars(obj.coordinate_names{i}),';']);
        end
        t = msg.timestamp/1000;
      end
    end
    
    function msg = encode(obj,t,x)
      sizecheck(t,1);
      sizecheck(x,[length(obj.coordinate_names),1]);
      msg = obj.lcmtype.newInstance();
      msg.timestamp = t*1000;
      for i=1:length(obj.coordinate_names)
        eval(['msg.',CoordinateFrame.stripSpecialChars(obj.coordinate_names{i}),' = x(',num2str(i),');']);
      end
    end
  end
  
  properties 
    lcmtype;
    lcmtype_constructor;
    coordinate_names={};
  end
end
  