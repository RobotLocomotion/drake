classdef DubinsPlant < DrakeSystem
% Defines the dynamics for the Dubins car/unicycle model.
  
  properties
    v = 2; % speed of the car
  end
  
  methods
    function obj = DubinsPlant()
      obj = obj@DrakeSystem(3,0,1,3,0,1);
      obj = setOutputFrame(obj,getStateFrame(obj));  % allow full state feedback
    end
    
    function [xdot, df, d2f, d3f] = dynamics(obj,t,x,u)      
        theta = x(3);
        xdot = [obj.v*cos(theta);  obj.v*sin(theta); u(1)];
        
        if (nargout>1)
            [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
        end
    end
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = [0 0 0]';
    end
    
    
    function [utraj,xtraj]=runDircol(p,x0,xf,tf0)

      con.u.lb = 0.5*p.umin; % Input saturations
      con.u.ub = 0.5*p.umax;
      con.x0.lb = x0; % Initial state
      con.x0.ub = x0;
      con.xf.lb = xf; % Final desired state
      con.xf.ub = xf;
      con.T.lb = 0.1; % Lower limit on how much time the trajectory should last for
      con.T.ub = 1; % Upper limit on how long the trajectory should last for

      options.method='dircol';
      %options.grad_test = true;
      
      function [g,dg] = cost(t,x,u)
        R = 0;
        g = u'*R*u;
        %g = sum((R*u).*u,1);
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        dg = zeros(1, 1 + size(x,1) + size(u,1));
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
      end
      

      info = 0;
      while (info~=1)
        % generate a random trajectory
        utraj0 = PPTrajectory(foh(linspace(0,tf0,11),0*randn(1,11)));
        
        tic
        [utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
        toc
      end


    end
          
  end
  
end
