classdef PendulumLCMCoder < LCMCoder
% PendulumLCMCoder
%   Codes and Decodes pendulum-specific LCM smessages 

  methods
    function str = getRobotName(obj)
      str = 'Pendulum';
    end
    
    function msg = encodeState(obj,t,x)
      msg = robotlib.examples.Pendulum.lcmt_pendulum_state();
      msg.timestamp = t;
      msg.theta = x(1);
      msg.thetaDot = x(2);
    end
    
    function [x,t] = decodeState(obj,msg)
      x = [msg.theta; msg.thetaDot];
    end
    
    function msg = encodeInput(obj,t,u)
      msg = robotlib.examples.Pendulum.lcmt_pendulum_input();
      msg.timestamp = t;
      msg.tau = u(1);
    end
    
    function [u,t] = decodeInput(obj,msg)
      u = msg.tau;
    end
  end
end
