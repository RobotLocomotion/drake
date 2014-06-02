classdef WarningManager < handle
  % carry this object around (e.g. in a class) to allow logic like 
  % displaying a warning only once (per instance of that class)
  
  properties
    disabled_msgids={};
  end
  
  methods
    function warnOnce(obj,msgid,varargin)
      if ~any(strcmp(msgid,obj.disabled_msgids))
        status = warning('query',msgid);
        if strcmp(status.state,'on')
          warning(msgid,varargin{:});
          obj.disabled_msgids{end+1} = msgid;
        end
      end
    end
    
    function tf = isDisabled(obj,msgid)
      tf = any(strcmp(msgid,obj.disabled_msgids));
    end
    
    function disableWarning(obj,msgid)
      obj.disabled_msgids{end+1} = msgid;
    end
  end
  
end