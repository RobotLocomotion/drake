classdef RimlessWheelPlant < HybridDrakeSystem
  
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
      obj = obj@HybridDrakeSystem(0,3);
      sys = RimlessWheelStancePlant(obj.m,obj.g,obj.l);
      obj = setInputFrame(obj,sys.getInputFrame);
      obj = setOutputFrame(obj,sys.getOutputFrame);
      obj = addMode(obj,sys);
      
      fc1=inline('x(1)-obj.gamma+obj.alpha','obj','t','x','u');  % theta<=gamma-alpha
      fc2=inline('x(2)','obj','t','x','u'); % thetadot<=0
      obj = addTransition(obj,1,andGuards(obj,fc1,fc2),@forwardCollisionDynamics,false,true);

      rc1=inline('obj.gamma+obj.alpha-x(1)','obj','t','x','u');  % theta>=gamma+alpha
      rc2=inline('-x(2)','obj','t','x','u'); % thetadot>=0
      obj = addTransition(obj,1,andGuards(obj,rc1,rc2),@reverseCollisionDynamics,false,true);

%      obj.ode_options = odeset('InitialStep',1e-3, 'Refine',1,'MaxStep',0.02);
    end

    function [xn,m,status] = forwardCollisionDynamics(obj,m,t,x,u)
      xn = [obj.gamma + obj.alpha; 
        x(2)*cos(2*obj.alpha);
        x(3) - 2*obj.l*sin(obj.alpha)];
      
      if (abs(xn(2))<0.01) status = 1;  % stop simulating if wheel is stopped
      else status = 0;
      end
    end          
    
    function [xn,m,status] = reverseCollisionDynamics(obj,m,t,x,u)
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

%      v.axis = [0 10 -1 2];
      xtraj = simulate(r,[0 10]);
      playback(v,xtraj);
    end
  end
  
end
