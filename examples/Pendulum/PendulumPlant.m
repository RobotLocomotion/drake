classdef PendulumPlant < SecondOrderSystem
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
      obj = obj@SecondOrderSystem(1,1,true);

      obj = setInputFrame(obj,PendulumInput);
      torque_limit = 3;
      obj = setInputLimits(obj,-torque_limit,torque_limit);
      
      obj = setStateFrame(obj,PendulumState);
      obj = setOutputFrame(obj,PendulumState);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = (u - obj.m*obj.g*obj.lc*sin(q) - obj.b*qd)/obj.I;
    end
    
    function [f,df,d2f,d3f]=dynamics(obj,t,x,u)
      f=dynamics@SecondOrderSystem(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x = getInitialState(obj)
      % Start me anywhere!
      x = randn(2,1);
    end
  end  
  
  methods 
    function [c,V]=balanceLQR(obj)
      x0=[pi;0]; u0=0;
      Q = diag([10 1]); R = 1;
      if (nargout<2)
        c = tilqr(obj,x0,u0,Q,R);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V] = tilqr(obj,x0,u0,Q,R);
        pp = feedback(obj.taylorApprox(0,x0,u0,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
      end
    end
    
    function [utraj,xtraj]=swingUpTrajectory(obj)
      x0 = [0;0]; tf0 = 4; xf = [pi;0];

      con.u.lb = obj.umin;
      con.u.ub = obj.umax;
      %con.u0.lb = 0;
      %con.u0.ub = 0;
      %con.uf.lb = 0;
      %con.uf.ub = 0;
      con.x0.lb = x0;
      con.x0.ub = x0;
      con.xf.lb = xf;
      con.xf.ub = xf;
      con.T.lb = 2;
      con.T.ub = 6;
      
      options.method='dircol';
      %options.grad_method='numerical';
      %options.grad_method={'user','numerical'};
      
      function [g,dg] = cost(t,x,u);
        R = 10;
        g = (R*u).*u;
        
        if (nargout>1)
          dg = [zeros(1,3),2*u'*R];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        if (nargout>1)
          dh = [1, zeros(1,2)];
        end
      end
      
      info=0;
      while (info~=1)
        utraj0 = PPTrajectory(foh(linspace(0,tf0,21),randn(1,21)));
        tic
        %options.grad_test = true;
        [utraj,xtraj,info] = trajectoryOptimization(obj,@cost,@finalcost,x0,utraj0,con,options);
        toc
      end
    end
    
    function c=trajectorySwingUpAndBalance(obj)
      [ti,Vf] = balanceLQR(obj);
      Vf = PolynomialLyapunovFunction(Vf.getFrame,Vf.getPoly*5);  % artificially prune, since ROA is solved without input limits

%      c = LQRTree(ti,Vf);
      [utraj,xtraj]=swingUpTrajectory(obj);  
      
      Q = diag([10 1]);  R=1;
      [tv,Vtraj] = tvlqr(obj,xtraj,utraj,Q,R,Vf);
      u0traj = PPTrajectory(zoh(xtraj.getTimeSpan,[0 0])); u0traj=setOutputFrame(u0traj,utraj.getOutputFrame);
      psys = taylorApprox(feedback(obj,tv),xtraj,u0traj,3);
      options.degL1=2;
      Vtraj=sampledFiniteTimeVerification(psys,Vf,Vtraj,xtraj.getBreaks(),xtraj,utraj,options);

      c = c.addTrajectory(tv,Vtraj);
    end
    
    function c=balanceLQRTree(p)
      xG=[pi;0]; uG=0;
      Q = diag([10 1]); R = 1;

      options.num_branches=5;
      %options.verify=false;
      options.xs = [0;0];
      options.Tslb = 2;
      options.Tsub = 6;
      options.degL1=4;
      c = LQRTree.buildLQRTree(p,xG,uG,@()rand(2,1).*[2*pi;10]-[pi;5],Q,R,options);
    end

  end

  methods(Static)
    function runPassive()  
    %% runs the passive system
      pd = PendulumPlant;
      pv = PendulumVisualizer(pd);
      traj = simulate(pd,[0 5],randn(2,1));
      playback(pv,traj);
    end
    
    function runLQR()
    %% run the lqr controller from a handful of initial conditions
      pd = PendulumPlant;
      pv = PendulumVisualizer(pd);
      c = balanceLQR(pd);
      sys = feedback(pd,c);
      for i=1:5
        xtraj=simulate(sys,[0 4],[pi;0]+0.2*randn(2,1));
        pv.playback(xtraj);
      end
    end
    
    function runLQRTest()
    %% run the lqr controller from a handful of initial conditions on the
    %% boundary of the estimated ROA and verify that it gets to the top
      pd = PendulumPlant;
      pd = pd.setInputLimits(-inf,inf);  % for now
      [c,V] = balanceLQR(pd);
      sys = feedback(pd,c);
      n=10;
      x0=[pi;0];
      V = V.inFrame(pd.getStateFrame);
      y=getLevelSet(V,[],struct('num_samples',n));
      for i=1:n
        xtraj=simulate(sys,[0 4],.01*x0 + .99*y(:,i));
        if (V.eval(4,xtraj.eval(4))>V.eval(0,xtraj.eval(0)))
          xtraj.eval(4)-x0
          error('simulation appears to be going uphill on the Lyapunov function');
        end
      end
    end
    
    function runDLQR()
    %% runs lqr on a sampled-data version of the plant dynamics
      pd = sampledData(PendulumPlant,.1);
      pv = PendulumVisualizer(pd);
      x0=[pi;0]; u0=0;
      Q = diag([10 1]); R = 1;
      c = tilqr(pd,x0,u0,Q,R);
      sys = feedback(pd,c);
      for i=1:5
        xtraj=simulate(sys,[0 4],[pi;0]+0.2*randn(2,1));
        pv.playback(xtraj);
      end
    end  
    
    function runSwingup()
    %% runs trajectory optimization and animates open-loop playback
      pd = PendulumPlant;
      pv = PendulumVisualizer(pd);
      utraj = swingUpTrajectory(pd);
      sys = cascade(utraj,pd);
      xtraj=simulate(sys,utraj.tspan,[0;0]);
      pv.playback(xtraj);
    end
    
    function runTrajectorySwingUpAndBalance()
    %% tilqr + funnel at the top, traj opt + tvlqr + funnel for swingup
      pd = PendulumPlant;
      pd = pd.setInputLimits(-inf,inf);  % for now
      pv = PendulumVisualizer(pd);
      c = trajectorySwingUpAndBalance(pd);
      sys = feedback(pd,c);
      for i=1:5
        xtraj=simulate(sys,[0 6]);
        pv.playback(xtraj);
      end
    end
  end
end
