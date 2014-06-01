classdef WarningManager < handle
  % carry this object around (e.g. in a class) to allow logic like 
  % displaying a warning only once (per instance of that class)
  
  properties
    disabled_msgids={};
  end
  
  methods
    function warnOnce(obj,msgid,varargin)
      if ~any(strcmp(msgid,obj.disabled_msgids))
        warning(msgid,varargin{:});
        obj.disabled_msgids{end+1} = msgid;
      end
    end
  end
  
end