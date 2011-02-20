classdef PendulumPlant < SecondOrderPlant
% Defines the dynamics for the Pendulum.
  
  properties
    m = 1;   % kg
    l = .5;  % m
    b = 0.1; % kg m^2 /s
    lc = .5; % m
    I = .25; %m*l^2; % kg*m^2
    g = 9.8; % m/s^2
  end
  
  methods
    function obj = PendulumPlant()
      % Construct a new PendulumPlant
      obj = obj@SecondOrderPlant(1,1,true);
      torque_limit = 3;
      obj = setInputLimits(obj,-torque_limit,torque_limit);
      obj = setAngleFlags(obj,0,[1;0],[1;0]);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = (u - obj.m*obj.g*obj.lc*sin(q) - obj.b*qd)/obj.I;
    end
    
    function df = sodynamicsGradients(obj,t,q,qd,u,order)
      % Implement the Taylor expansion of the second-order gradients
      if (nargin>4 && order>3) df = sodynamicsGradients@SecondOrderPlant(obj,t,x,u,order); end
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
      % Start me anywhere!
      x = randn(2,1);
    end
  end  

  methods(Static)
    function run()  % runs the passive system
      pd = PendulumPlant;
      pv = PendulumVisualizer;
      traj = simulate(pd,[0 5],randn(2,1));
      playback(pv,traj);
    end
  end
end