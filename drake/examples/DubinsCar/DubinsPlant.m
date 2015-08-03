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

      N = 21;
      prog = DircolTrajectoryOptimization(p,N,[.1 1]);
      prog = prog.addInputConstraint(BoundingBoxConstraint(.5*p.umin,.5*p.umax),1:N);
      prog = prog.addStateConstraint(ConstantConstraint(x0),1);
      prog = prog.addStateConstraint(ConstantConstraint(xf),N);
      prog = prog.addRunningCost(@cost);
      prog = prog.addFinalCost(@finalCost);

      function [g,dg] = cost(dt,x,u)
        R = 0;
        g = u'*R*u;
        %g = sum((R*u).*u,1);
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        dg = zeros(1, 1 + size(x,1) + size(u,1));
      end
      
      function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
      end

      traj_init.x = PPTrajectory(foh([0,tf0],[x0,xf]));
      info = 0;
      while (info~=1)
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0);
        toc
      end


    end
          
  end
  
end
