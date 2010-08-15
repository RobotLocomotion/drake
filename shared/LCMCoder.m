classdef LCMCoder
% Interface for classes which encode and decode robot-specific messages from LCM.

  methods 
    str = getRobotName(obj)
    
    msg = encodeX(obj,t,x)
    [x,t] = decodeX(obj,msg)

    msg = encodeU(obj,t,u)
    [u,t] = decodeU(obj,msg)
  end

  methods
    function msg = encodeState(obj,t,x)
      error('please use encodeX now');
    end
    function [x,t] = decodeState(obj,msg)
      error('please use decodeX now');
    end
    function msg = encodeInput(obj,t,u)
      error('please use encodeU now');
    end
    function [u,t] = decodeInput(obj,msg)
      error('please use decodeU now');
    end
  end
end
