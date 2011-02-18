classdef RimlessWheelPlant < FiniteStateMachine
  
  properties
    m = 1;
    l = 1;
    g = 9.81;
    alpha = pi/8;
    %gamma = 0.03;  % standing is only fixed point
    gamma = 0.08;  % standing and rolling fixed points
    %gamma=5*pi/180;
    %gamma = alpha+0.01;  % only rolling fixed point
  end
  
  methods 
    function obj = RimlessWheelPlant()
      obj = obj@FiniteStateMachine();
      sys = RimlessWheelStancePlant();
      sys.m = obj.m;
      sys.l = obj.l;
      sys.g = obj.g;
      obj = addMode(obj,sys);
      
      fc1=inline('x(1)-obj.gamma+obj.alpha','obj','t','x','u');  % theta<=gamma-alpha
      fc2=inline('x(2)','obj','t','x','u'); % thetadot<=0
      obj = addTransition(obj,1,1,and_guards(obj,fc1,fc2),@forward_collision_dynamics,false,true);

      rc1=inline('obj.gamma+obj.alpha-x(1)','obj','t','x','u');  % theta>=gamma+alpha
      rc2=inline('-x(2)','obj','t','x','u'); % thetadot>=0
      obj = addTransition(obj,1,1,and_guards(obj,rc1,rc2),@reverse_collision_dynamics,false,true);

%      obj.ode_options = odeset('InitialStep',1e-3, 'Refine',1,'MaxStep',0.02);
    end

    function [xn,status] = forward_collision_dynamics(obj,t,x,u)
      xn = [obj.gamma + obj.alpha; 
        x(2)*cos(2*obj.alpha);
        x(3) - 2*obj.l*sin(obj.alpha)];
      
      if (abs(xn(2))<0.01) status = 1;  % stop simulating if wheel is stopped
      else status = 0;
      end
    end          
    
    function [xn,status] = reverse_collision_dynamics(obj,t,x,u)
      xn = [obj.gamma - obj.alpha; 
        x(2)*cos(2*obj.alpha);
        x(3) + 2*obj.l*sin(obj.alpha)];
      
      if (abs(xn(2))<0.01) status = 1;  % stop simulating if wheel is stopped
      else status = 0;
      end
    end          
  end

  methods (Static)
    function run()
      r = RimlessWheelPlant();
      v = RimlessWheelVisualizer(r);

      xtraj = simulate(r,[0 10]);
      playback(v,xtraj);
    end
  end
  
end
