classdef CartPoleLCMCoder < LCMCoder
% Encodes and Decodes pendulum-specific LCM smessages 

  methods
    function str = getRobotName(obj)
      % robot name
      str = 'CartPole';
    end
    
    function msg = encodeX(obj,t,x)
      % encodes the state message
      msg = robotlib.examples.CartPole.lcmt_cartpole_x();
      msg.timestamp = t*1000;
      msg.x = x(1);
      msg.theta = x(2);
      msg.xdot = x(3);
      msg.thetaDot = x(4);
    end
    
    function [x,t] = decodeX(obj,msg)
      % decodes the state message
      msg = robotlib.examples.CartPole.lcmt_cartpole_x(msg.data);
      x = [msg.x; msg.theta; msg.xdot; msg.thetaDot];
      t = msg.timestamp/1000;
    end
    
    function msg = encodeU(obj,t,u)
      % encodes the input message
      msg = robotlib.examples.CartPole.lcmt_cartpole_u();
      msg.timestamp = t*1000;
      msg.f = u(1);
    end
    
    function [u,t] = decodeU(obj,msg)
      % decodes the input message
      msg = robotlib.examples.CartPole.lcmt_cartpole_u(msg.data);
      u = msg.f;
      t = msg.timestamp/1000;
    end
  end
end
