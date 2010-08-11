classdef PerchingGliderLCMCoder < LCMCoder
% Encodes and Decodes glider-specific LCM smessages 

  methods
    function str = getRobotName(obj)
      % robot name
      str = 'PerchingGlider';
    end
    
    function msg = encodeState(obj,t,x)
      % encodes the state message
      msg = robotlib.examples.PerchingGlider.lcmt_glider_state();
      msg.timestamp = t*1000;
      msg.x = x(1);
      msg.z = x(2);
      msg.theta = x(3);
      msg.phi = x(4);
      msg.xdot = x(5);
      msg.zdot = x(6);
      msg.thetadot = x(7);
    end
    
    function [x,t] = decodeState(obj,msg)
      % decodes the state message
      msg = robotlib.examples.Pendulum.lcmt_glider_state(msg.data);
      t = msg.timestamp/1000;
      x = [msg.x; msg.z; msg.theta; msg.phi; msg.xdot; msg.zdot; msg.thetaDot];
    end
    
    function msg = encodeInput(obj,t,u)
      % encodes the input message
      msg = robotlib.examples.Pendulum.lcmt_glider_input();
      msg.timestamp = t*1000;
      msg.phidot = u(1);
    end
    
    function [u,t] = decodeInput(obj,msg)
      % decodes the input message
      msg = robotlib.examples.Pendulum.lcmt_glider_input(msg.data);
      t = msg.timestamp/1000;
      u = msg.phidot;
    end
  end
end
