classdef PendulumPlant < SecondOrderSystem
% Defines the dynamics for the Pendulum.
  
  properties
    m = 1;   % kg
    l = .5;  % m
    b = 0.1; % kg m^2 /s
    lc = .5; % m
    I = .25; %m*l^2; % kg*m^2
    g = 9.81; % m/s^2
    
    xG;
    uG;
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
      
      obj.xG = Point(PendulumState,[pi;0]);
      obj.uG = Point(PendulumInput,0);
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
      Q = diag([10 1]); R = 1;
      if (nargout<2)
        options.angle_flag = [1;0];
        c = tilqr(obj,obj.xG,obj.uG,Q,R,options);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V] = tilqr(obj,obj.xG,obj.uG,Q,R);
        pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
       
        % construct the wrapping frame after verification (because it's not
        % smooth, and otherwise interferes)
        c = setInputFrame(c,c.getInputFrame.constructFrameWithAnglesWrapped([1;0]));
        V=setFrame(V,c.getInputFrame);
      end
    end
    
    function [utraj,xtraj]=swingUpTrajectory(obj,options)
      x0 = [0;0]; tf0 = 4; xf = double(obj.xG);

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
      
      if nargin<2, options=struct(); end
      if ~isfield(options,'method') options.method='dircol'; end
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
      Vf = 5*Vf;  % artificially prune, since ROA is solved without input limits

      c = LQRTree(obj.xG,obj.uG,ti,Vf);
      [utraj,xtraj]=swingUpTrajectory(obj);  
      
      Q = diag([10 1]);  R=1;
      [tv,Vswingup] = tvlqr(obj,xtraj,utraj,Q,R,Vf);
      psys = taylorApprox(feedback(obj,tv),xtraj,[],3);
      options.degL1=2;
      Vswingup=sampledFiniteTimeVerification(psys,xtraj.getBreaks(),Vf,Vswingup,options);

      c = c.addTrajectory(xtraj,utraj,tv,Vswingup);
    end
    
    function c=balanceLQRTree(obj)
      Q = diag([10 1]); R = 1;

      options.num_branches=5;
%      options.verify=false;
      options.xs = [0;0];
      options.Tslb = 2;
      options.Tsub = 6;
      options.degL1=4;
      c = LQRTree.buildLQRTree(obj,obj.xG,obj.uG,@()rand(2,1).*[2*pi;10]-[pi;5],Q,R,options);
    end

  end

end
