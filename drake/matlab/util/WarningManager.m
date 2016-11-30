classdef WarningManager < handle
  % carry this object around (e.g. in a class) to allow logic like 
  % displaying a warning only once (per instance of that class)
  
  properties
    disabled_msgids;
  end
  
  methods
    function obj = WarningManager()
      obj.disabled_msgids=containers.Map;
    end
    function warnOnce(obj,msgid,varargin)
      % call this instead of warning to print the warning 
      % only once.  after calling, the message is 'disabled'
      % @param msgid as in warning msgid
      % @param varargin message string + additional info
      if ~isKey(obj.disabled_msgids,msgid)
        status = warning('query',msgid);
        if strcmp(status.state,'on')
          warning(msgid,varargin{:});
          obj.disabled_msgids(msgid) = true;
        end
      end
    end
    
    function tf = isDisabled(obj,msgid)
      % check if the message is already 'disabled'
      % @param msgid as in warning msgid
      tf = isKey(obj.disabled_msgids,msgid);
    end
    
    function disableWarning(obj,msgid)
      % disable this message type without printing the warning
      % @param msgid as in warning msgid
      obj.disabled_msgids(msgid) = true;
    end
  end
  
end
