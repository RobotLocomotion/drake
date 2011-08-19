classdef AcrobotLCMCoder < LCMCoder
% Encodes and Decodes acrobot-specific LCM smessages 

  methods
    function obj = AcrobotLCMCoder
      obj = obj@LCMCoder(4,1,4);
    end
    
    function str = getRobotName(obj)
      % robot name
      str = 'Acrobot';
    end
    
    function msg = encodeX(obj,t,x)
      % encodes the state message
      msg = robotlib.examples.Acrobot.lcmt_acrobot_x();
      msg.timestamp = t*1000;
      msg.theta1 = x(1);
      msg.theta1Dot = x(3);
      msg.theta2 = x(2);
      msg.theta2Dot = x(4);
    end
    
    function [x,t] = decodeX(obj,msg)
      % decodes the state message
      msg = robotlib.examples.Acrobot.lcmt_acrobot_x(msg.data);
      x = [msg.theta1; msg.theta2; msg.theta1Dot; msg.theta2Dot];
      t = msg.timestamp/1000;
    end
    
    function msg = encodeU(obj,t,u)
      % encodes the input message
      msg = robotlib.examples.Acrobot.lcmt_acrobot_u();
      msg.timestamp = t*1000;
      msg.tau = u(1);
    end
    
    function [u,t] = decodeU(obj,msg)
      % decodes the input message
      msg = robotlib.examples.Acrobot.lcmt_acrobot_u(msg.data);
      u = msg.tau;
      t = msg.timestamp/1000;
    end
    
    function msg = encodeY(obj,t,y)
      msg = robotlib.examples.Acrobot.lcmt_acrobot_y();
      msg.timestamp = t*1000;
      msg.theta1 = y(1);
      msg.theta2 = y(2);
      warning('tau as output is not implemented yet'); 
      msg.tau = 0;  
    end
    
    function [y,t] = decodeY(obj,msg)  
      msg = robotlib.examples.Acrobot.lcmt_acrobot_y(msg.data);
      y = [msg.theta1; msg.theta2; 0; 0]; 
      warning('thetadot not implemented yet'); 
      t = msg.timestamp/1000;
    end
  end
end
