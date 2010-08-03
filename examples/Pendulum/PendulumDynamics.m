classdef PendulumDynamics < SecondOrderDynamics

  properties
    m = 1;   % kg
    l = .5;  % m
    b = 0.1; % kg m^2 /s
    lc = .5; % m
    I = .25; %m*l^2; % kg*m^2
    g = 9.8; % m/s^2
  end
  
  methods
    function obj = PendulumDynamics()
      obj = obj@SecondOrderDynamics(1,1);
      torque_limit = 3;
      obj = setInputLimits(obj,-torque_limit,torque_limit);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      qdd = (u - obj.m*obj.g*obj.lc*sin(q) - obj.b*qd)/obj.I;
    end
    
    function df = sodynamicsGradients(obj,t,q,qd,u,order)
      if (nargin>4 && order>3) df = sodynamicsGradients@SecondOrderDynamics(obj,t,x,u,order); end
      if (nargin<5) order=1; end
      
      dfdt = 0;
      dfdq = -obj.m*obj.g*obj.lc*cos(q)/obj.I;
      dfdqd = -obj.b/obj.I;
      dfdu = 1/obj.I;
      df{1} = [dfdt, dfdq, dfdqd, dfdu];
      
      if (order>1)
        df{2}{1} = zeros(1,4);  %df{1}dt
        df{2}{2} = zeros(1,4);  df{2}{2}(1,2) = obj.m*obj.g*obj.lc*sin(q)/obj.I; % df{1}dq
        df{2}{3} = zeros(1,4);  %df{1}dqdot
        df{2}{4} = zeros(1,4);  %df{1}du

        if (order>2)
          for i=1:4; for j=1:4; df{3}{i}{j} = zeros(1,4); end, end %df{2}{i}dz(j) where z=[t,x,u];
          df{3}{2}{2}(1,2) = obj.m*obj.g*obj.lc*cos(q)/obj.I;  %df{2}{2}(2,2)dq
        end
      end
    end
    
    function x = getInitialState(obj)
      x = randn(2,1);
    end
    
  end  
  
end