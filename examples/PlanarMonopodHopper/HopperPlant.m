classdef HopperPlant < DrakeSystem
  
% state variables:
%   q(1) - x position of the foot
%   q(2) - y position of the foot
%   q(3) - absolute angle of leg (from vertical)
%   q(4) - absolute angle of body
%   q(5) - leg length
%   q(6:10) - derivatives of q(1:5)

% control variables:
%   u(1) - position of leg spring actuator
%   u(2) - torque at the hip

  
  properties
    m = 10.0;        % mass of the body
    m_l = 1.0;       % mass of the leg
    J = 10.0;        % moment of inertia of the body
    J_l = 1.0;       % moment of inertia of the leg
    g = 9.8;         % gravity
    k_l = 1e3;       % spring constant of leg spring
    k_stop = 1e5;    % spring constant of leg stop
    b_stop = 1e3;    % damping constant of leg stop
    k_g = 1e4;       % spring constant of the ground
    b_g = 300.0;     % damping constant of the ground
    r_s0 = 1.0;      % rest length of the leg spring
    l_1 = 0.5;       % distance from the foot to the com of the leg
    l_2 = 0.4;       % distance from the hip to the com of the body
  end
  
  methods
    function obj = HopperPlant()
      obj = obj@DrakeSystem(10,0,2,10,false,true);
      obj = setNumZeroCrossings(obj,3);
%      obj = setInputLimits(obj,[-inf;-50],[inf;50]);  % see bug 1022
      obj = setOutputFrame(obj,getStateFrame(obj));  % allow full state feedback
    end
    
    function x0 = getInitialState(obj)
      x0 = [0.0; 0.4; 0.01; 0.0; 1.0; zeros(5,1)];
      %x0 = [ 0.0; rand(1,1); 0.5*randn(8,1) ];
    end
    
    function zcs = zeroCrossings(obj,t,x,u)
      zcs = [obj.r_s0 - x(5) + u(1); ... % r_sd==0
        x(2); ...  % x(2)==0
        -obj.k_g*x(2) - obj.b_g*x(7)];  % F_z==0 (only actually want this to trigger when x(2)<0, but ok for now)
    end
    
    function xdot = dynamics(obj,t,x,u)
      % These equations were taken (with minor corrections) from Raibert's
      % book _Legged Robots That Balance_, p. 172.
      
      R = x(5) - obj.l_1;
      s1 = sin(x(3));
      c1 = cos(x(3));
      s2 = sin(x(4));
      c2 = cos(x(4));

      r_sd = obj.r_s0 - x(5) + u(1);
      if (r_sd > 0)  
        F_k = obj.k_l*r_sd;
      else
        F_k = obj.k_stop*r_sd - obj.b_stop*x(10);
      end

      if (x(2) < 0)
        F_x = -obj.b_g*x(6);  % don't simulate k_g in horizontal direction
                          % (want autonomous dynamics)
        F_z = max(obj.k_g*(- x(2)) - obj.b_g*x(7),0.0);
      else
        F_x = 0.0;
        F_z = 0.0;
      end

      a = obj.l_1*F_z*s1 - obj.l_1*F_x*c1 - u(2);

      M = [ -obj.m_l*R, 0, (obj.J_l-obj.m_l*R*obj.l_1)*c1, 0, 0;
        0, obj.m_l*R, (obj.J_l-obj.m_l*R*obj.l_1)*s1, 0, 0;
        obj.m*R, 0, (obj.J_l+obj.m*R*x(5))*c1, obj.m*R*obj.l_2*c2, obj.m*R*s1;
        0, -obj.m*R, (obj.J_l+obj.m*R*x(5))*s1, obj.m*R*obj.l_2*s2, -obj.m*R*c1;
        0, 0, obj.J_l*obj.l_2*cos(x(3)-x(4)), -obj.J*R, 0 ];

      eta = [ a*c1 - R*(F_x - F_k*s1 - obj.m_l*obj.l_1*x(8)*x(8)*s1);
        a*s1 + R*(obj.m_l*obj.l_1*x(8)*x(8)*c1 + F_z - F_k*c1 - obj.m_l*obj.g);
        a*c1 + R*F_k*s1 + obj.m*R*(x(5)*x(8)*x(8)*s1 + obj.l_2*x(9)*x(9)*s2 - 2*x(10)*x(8)*c1);
        a*s1 - R*(F_k*c1 - obj.m*obj.g) - obj.m*R*(2*x(10)*x(8)*s1 + x(5)*x(8)*x(8)*c1 + obj.l_2*x(9)*x(9)*c2);
        a*obj.l_2*cos(x(3)-x(4)) - R*(obj.l_2*F_k*sin(x(4)-x(3)) + u(2)) ];
	
      qdd = M\eta;
      xdot = [x(6:10);qdd];
    end
    
    function y=output(obj,t,x,u)
      y=x;
    end
  end
  
  
end
  
  
