classdef PendulumLCMCoder < LCMCoder
% Encodes and Decodes pendulum-specific LCM smessages 

  methods
    function obj = PendulumLCMCoder()
      obj = obj@LCMCoder(2,1,2);
    end
    
    function str = getRobotName(obj)
      % robot name
      str = 'Pendulum';
    end
    
    function msg = encodeX(obj,t,x)
      % encodes the state message
      msg = drake.examples.Pendulum.lcmt_pendulum_x();
      msg.timestamp = t*1000;
      msg.theta = x(1);
      msg.thetaDot = x(2);
    end
    
    function [x,t] = decodeX(obj,msg)
      % decodes the state message
      msg = drake.examples.Pendulum.lcmt_pendulum_x(msg.data);
      x = [msg.theta; msg.thetaDot];
      t = msg.timestamp/1000;
    end
    
    function msg = encodeU(obj,t,u)
      % encodes the input message
      msg = drake.examples.Pendulum.lcmt_pendulum_u();
      msg.timestamp = t*1000;
      msg.tau = u(1);
    end
    
    function [u,t] = decodeU(obj,msg)
      % decodes the input message
      msg = drake.examples.Pendulum.lcmt_pendulum_u(msg.data);
      u = msg.tau;
      t = msg.timestamp/1000;
    end
    
    function msg = encodeY(obj,t,y)
      % encodes the input message
      msg = Pendulum.lcmt_pendulum_y();
      msg.timestamp = t*1000;
      msg.theta = y(1);
      msg.tau = y(2);
    end
    
    function [y,t] = decodeY(obj,msg)
      % decodes the input message
      msg = Pendulum.lcmt_pendulum_y(msg.data);
      y(1) = msg.theta;
      y(2) = msg.tau;
      t = msg.timestamp/1000;
    end
  end
end
