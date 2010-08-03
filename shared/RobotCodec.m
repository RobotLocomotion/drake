classdef RobotCodec
% RobotCodec
%   Codes and Decodes input and state messages from network packets for
%   LCM, ROS, etc.

  methods 
    str = getRobotName(obj)
    
    msg = encodeLCMState(obj,x)
    x = decodeLCMState(obj,msg)

    msg = encodeLCMInput(obj,u)
    u = decodeLCMInput(obj,msg)
  end
  
end
