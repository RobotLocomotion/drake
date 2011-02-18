classdef CompassGaitPlant < FiniteStateMachine
  
  properties
    m = 5;
    mh = 10;
    a = 0.5;
    b = 0.5;
    g = 9.8;
    l;
    gamma = 3*pi/180;
  end
  
  methods 
    function obj = CompassGaitPlant()
      obj.l=obj.a+obj.b;
      p = CompassGaitStancePlant(obj.m,obj.mh,obj.a,obj.b,obj.l,obj.g);
      obj = obj.addMode(p);
      obj = obj.addMode(p);

      obj = addTransition(obj,1,2,and_guards(obj,@foot_collision_guard_1,@foot_collision_guard_2),@collision_dynamics,false,true);
      obj = addTransition(obj,2,1,and_guards(obj,@foot_collision_guard_1,@foot_collision_guard_2),@collision_dynamics,false,true);

      %      obj.ode_options = odeset('InitialStep',1e-3, 'Refine',1,'MaxStep',0.02);
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
      obj = setInputLimits(obj,-50,50);
    end
    
    function [g,dg] = foot_collision_guard_1(obj,t,x,u)
      g = x(1)+x(2)+2*obj.gamma;  % theta_st - gamma <= - (theta_sw - gamma) 
      dg{1} = [0,1,1,0,0,0];
    end
    
    function [g,dg] = foot_collision_guard_2(obj,t,x,u);
      g = -x(1);   % theta_st >= 0
      dg{1} = [0,-1,0,0,0,0];
    end
      
    function [xp,status,dxp] = collision_dynamics(obj,t,xm,u)
      m=obj.m; mh=obj.mh; a=obj.a; b=obj.b; l=obj.l;
      
      alpha = (xm(1) - xm(2));
      Qm = [ -m*a*b, -m*a*b + (mh*l^2 + 2*m*a*l)*cos(alpha); 0, -m*a*b ];
      Qp = [ m*b*(b - l*cos(alpha)), m*l*(l-b*cos(alpha)) + m*a^2 + mh*l^2; m*b^2, -m*b*l*cos(alpha) ];
      
      xp = [xm([2,1]); ...     % switch stance and swing legs
        Qp\Qm*xm(3:4)];   % inelastic impulse
      
      if (nargout>2)
        Qpi = inv(Qp);
        dalpha = [1 -1];  % d/dq
        dQpdalpha = [m*b*l*sin(alpha), m*l*b*sin(alpha); 0, m*b*l*sin(alpha) ];
        dQpidalpha = -Qpi*dQpdalpha*Qpi;
        dQmdalpha = [0, -2*m*a*l*sin(alpha); 0, 0];
        dxpdxm = zeros(4);
        dxpdxm(1:2,1:2) = [ 0 1 ; 1 0];
        dxpdxm(3:4,3:4) = Qpi*Qm;
        dxpdxm(3:4,1) = (dQpidalpha*dalpha(1))*Qm*xm(3:4) + Qpi*(dQmdalpha*dalpha(1))*xm(3:4);
        dxpdxm(3:4,2) = (dQpidalpha*dalpha(2))*Qm*xm(3:4) + Qpi*(dQmdalpha*dalpha(2))*xm(3:4);
        dxp{1} = [zeros(4,1),dxpdxm,zeros(4,1)];
      end
      status = 0;
    end
    
  end
  
  methods (Static)
    function run()
      r = CompassGaitPlant();
      v = CompassGaitVisualizer(r);
      traj = simulate(r,[0 10]);
      playback(v,traj);    
    end
  end  
  
end
