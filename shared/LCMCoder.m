classdef LCMCoder
% Interface for classes which encode and decode robot-specific messages from LCM.

  methods 
    str = getRobotName(obj)
    
    msg = encodeState(obj,t,x)
    [x,t] = decodeState(obj,msg)

    msg = encodeInput(obj,t,u)
    [u,t] = decodeInput(obj,msg)
  end
  
end
