classdef CartPolePlant < Manipulator

  properties
    mc = 10;   % mass of the cart in kg
    mp = 1;    % mass of the pole (point mass at the end) in kg
    l = 0.5;   % length of the pole in m
    g = 9.81;  % gravity m/s^2
    
    xG;
    uG;
  end
  
  methods
    function obj = CartPolePlant
      obj = obj@Manipulator(2,1);
      obj = setInputLimits(obj,-30,30);
      obj = setOutputFrame(obj,obj.getStateFrame);  % allow full-state feedback
      
      obj.xG = Point(obj.getStateFrame,[0;pi;0;0]);
      obj.uG = Point(obj.getInputFrame,0);
    end
        
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      mc=obj.mc;  mp=obj.mp;  l=obj.l;  g=obj.g;
      s = sin(q(2)); c = cos(q(2));

      H = [mc+mp, mp*l*c; mp*l*c, mp*l^2];
      C = [0 -mp*qd(2)*l*s; 0 0];
      G = [0; mp*g*l*s];
      B = [1; 0];
      
      C = C*qd + G;
    end
    
    function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x0 = getInitialState(obj)
      x0 = randn(4,1);
    end
    
  end

  methods 
    function [c,V]=balanceLQR(obj)
      Q = diag([1 50 1 50]);
      R = .2;

      if (nargout<2)
        c = tilqr(obj,obj.xG,obj.uG,Q,R);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V] = tilqr(obj,obj.xG,obj.uG,Q,R);
        pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
      end
    end
    
    function [c,V]=balanceHinf(obj)
      Bw = [zeros(2,2); eye(2)];  %uncertainty affects velocities only
      Q = diag([1 50 1 50]);
      R = .1;
      gamma = 20;
      if (nargout<2)
        c = tiHinf(obj,obj.xG,obj.uG,Q,R,Bw,gamma);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V]=tiHinf(obj,obj.xG,obj.uG,Q,R,Bw,gamma);
        pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
      end        
    end
    
    function [utraj,xtraj]=swingUpTrajectory(obj)
      x0 = zeros(4,1); tf0 = 4; xf = double(obj.xG);
      N = 21;
      
      obj = setInputLimits(obj,-inf,inf);
      prog = DircolTrajectoryOptimization(obj,N,[2 6]);
      prog = prog.addStateConstraint(ConstantConstraint(x0),1);
      prog = prog.addStateConstraint(ConstantConstraint(xf),N);
      prog = prog.addRunningCost(@cost);
      prog = prog.addFinalCost(@finalCost);
      
      function [g,dg] = cost(dt,x,u)
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
      end
      
      function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
      end
      
      traj_init.x = PPTrajectory(foh([0,tf0],[x0,xf]));
      for attempts=1:10
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        if info==1, break; end
      end
    end
    
    function c=trajectorySwingUpAndBalance(obj)
      [ti,Vf] = balanceLQR(obj);

%      c = LQRTree(ti,Vf);
      [utraj,xtraj]=swingUpTrajectory(obj);  
      
      Q=diag([10,10,1,1]); R=.1;
      [tv,Vtraj] = tvlqr(obj,xtraj,utraj,Q,R,Vf);
      psys = taylorApprox(feedback(obj,tv),xtraj,[],3);
      options.rho0_tau = 10;
      options.max_iterations = 3;
      Vtraj=sampledFiniteTimeVerification(psys,xtraj.getBreaks(),Vf,Vtraj,options);

%      c = c.addTrajectory(tv,Vtraj);
    end  
  end
end
