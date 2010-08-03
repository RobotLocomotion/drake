classdef PendulumCodec < RobotCodec
% PendulumCodec
%   Codes and Decodes input and state messages from network packets for
%   LCM, ROS, etc.

  methods
    function str = getRobotName(obj)
      str = 'Pendulum';
    end
    
    function msg = encodeLCMState(obj,x)
      msg = robotlib.robots.Pendulum.lcmt_pendulum_state();
      msg.timestamp = 0;
      msg.theta = x(1);
      msg.thetaDot = x(2);
    end
    
    function x = decodeLCMState(obj,msg)
      x = [msg.theta; msg.thetaDot];
    end
    
    function msg = encodeLCMInput(obj,u)
      msg = robotlib.robots.Pendulum.lcmt_pendulum_input();
      msg.timestamp = 0;
      msg.tau = u(1);
    end
    
    function u = decodeLCMInput(obj,msg)
      u = msg.tau;
    end
  end
end
