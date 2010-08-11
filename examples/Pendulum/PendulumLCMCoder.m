classdef PendulumLCMCoder < LCMCoder
% Encodes and Decodes pendulum-specific LCM smessages 

  methods
    function str = getRobotName(obj)
      % robot name
      str = 'Pendulum';
    end
    
    function msg = encodeState(obj,t,x)
      % encodes the state message
      msg = robotlib.examples.Pendulum.lcmt_pendulum_x();
      msg.timestamp = t*1000;
      msg.theta = x(1);
      msg.thetaDot = x(2);
    end
    
    function [x,t] = decodeState(obj,msg)
      % decodes the state message
      msg = robotlib.examples.Pendulum.lcmt_pendulum_x(msg.data);
      x = [msg.theta; msg.thetaDot];
      t = msg.timestamp/1000;
    end
    
    function msg = encodeInput(obj,t,u)
      % encodes the input message
      msg = robotlib.examples.Pendulum.lcmt_pendulum_u();
      msg.timestamp = t*1000;
      msg.tau = u(1);
    end
    
    function [u,t] = decodeInput(obj,msg)
      % decodes the input message
      msg = robotlib.examples.Pendulum.lcmt_pendulum_u(msg.data);
      u = msg.tau;
      t = msg.timestamp/1000;
    end
  end
end
