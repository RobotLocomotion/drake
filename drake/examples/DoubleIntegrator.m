classdef DoubleIntegrator < LinearSystem
  
  methods
    function obj = DoubleIntegrator
      obj = obj@LinearSystem([0,1;0,0],[0;1],[],[],eye(2),[]);
      obj = setInputLimits(obj,-1,1);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
    
    function g = mintime_cost(obj,x,u)
      g = ((x(1,:) ~= 0) | (x(2,:) ~= 0));
    end
    
    function g = lqr_cost(obj,x,u)
      Q=eye(2); R = 10;
      g = x'*Q*x + u'*R*u;
    end
    
    function v = constructVisualizer(obj)
      function draw(t,x)
        blockx = [-1, -1, 1, 1, -1];
        blocky = [0, 0.5, 0.5, 0, 0];
        
        % draw the mass
        brickcolor=[.75 .6 .5];
        fill(blockx+repmat(x(1),1,5),blocky,brickcolor);

        faintline=[.6 .8 .65]*1.1;
        plot(min(blockx)+[0 0],[-5 5],'k:','Color',faintline);
        plot(max(blockx)+[0 0],[-5 5],'k:','Color',faintline);

        % draw the ground
        line([-5, 5], [0, 0],'Color',[.3 .5 1],'LineWidth',1);
        axis equal;
        axis([-5 5 -1 2]);
        %grid on
      end
      
      v = FunctionHandleVisualizer(getOutputFrame(obj),@draw);
    end
  end
  
  methods (Static = true)
    function runValueIteration
      p = DoubleIntegrator;
      options.gamma = .9999;  % discount factor
      options.dt = .01;
      
      costfun = @mintime_cost;
%      costfun = @lqr_cost;
      xbins = {[-3:.2:3],[-4:.2:4]};
      ubins = linspace(p.umin,p.umax,9);
      mdp = MarkovDecisionProcess.discretizeSystem(p,costfun,xbins,ubins,options);
      
      function drawfun(J,PI)
        figure(2); clf;
        n1=length(xbins{1});
        n2=length(xbins{2});
        subplot(2,1,1);imagesc(xbins{1},xbins{2},reshape(ubins(PI),n1,n2)');
        axis xy;
        xlabel('q');
        ylabel('qdot');
        title('u(x)');
        subplot(2,1,2);imagesc(xbins{1},xbins{2},reshape(J,n1,n2)');
        axis xy;
        xlabel('q');
        ylabel('qdot');
        title('J(x)');
        drawnow;
        
%        pause;
      end
      valueIteration(mdp,0.0001,@drawfun);
      
      % add colorbars (it's expensive to do it inside the loop)
      figure(2); 
      subplot(2,1,1); colorbar;
      subplot(2,1,2); colorbar;
    end
    
    function runDirtran
      % Simple example of Direct Transcription trajectory optimization

      % create the system
      plant = DoubleIntegrator;
      
      % set up a direct transcription problem with N knot points and bounds on
      % the duration
      N = 20;
      prog = DirtranTrajectoryOptimization(plant,N,[0.1 10]);
      
      % add the initial value constraint
      x0 = [-2; -2];
      prog = addStateConstraint(prog,ConstantConstraint(x0),1);
      
      % add the final value constraint
      xf = [0;0];
      prog = addStateConstraint(prog,ConstantConstraint(xf),N);
      
      % add the cost function g(dt,x,u) = 1*dt
      function [g,dg] = cost(dt,x,u)
        g = dt; dg = [1,0*x',0*u']; % see geval.m for our gradient format
      end
      prog = addRunningCost(prog,@cost);
      
      % add a display function to draw the trajectory on every iteration
      function displayStateTrajectory(t,x,u)
        plot(x(1,:),x(2,:),'b.-','MarkerSize',10);
        axis([-5,1,-2.5,2.5]); axis equal;
        drawnow;
      end
      prog = addTrajectoryDisplayFunction(prog,@displayStateTrajectory);
      
      % solve the optimization problem (with 2 sec as the initial guess for
      % the duration)
      prog.solveTraj(2);
    end

    function runDircol
      % Simple example of Direct Collocation trajectory optimization

      % create the system
      plant = DoubleIntegrator;
      
      % set up a direct transcription problem with N knot points and bounds on
      % the duration
      N = 20;
      prog = DircolTrajectoryOptimization(plant,N,[0.1 10]);
      
      % add the initial value constraint
      x0 = [-2; -2];
      prog = addStateConstraint(prog,BoundingBoxConstraint(x0,x0),1);
      
      % add the final value constraint
      xf = [0;0];
      prog = addStateConstraint(prog,BoundingBoxConstraint(xf,xf),N);
      
      % add the cost function g(dt,x,u) = 1*dt
      function [g,dg] = cost(dt,x,u)
        g = dt; dg = [1,0*x',0*u']; % see geval.m for our gradient format
      end
      prog = addRunningCost(prog,@cost);
      
      % add a display function to draw the trajectory on every iteration
      function displayStateTrajectory(t,x,u)
        plot(x(1,:),x(2,:),'b.-','MarkerSize',10);
        axis([-5,1,-2.5,2.5]); axis equal;
        drawnow;
      end
      prog = addTrajectoryDisplayFunction(prog,@displayStateTrajectory);
      
      % solve the optimization problem (with 2 sec as the initial guess for
      % the duration)
      prog.solveTraj(2);
    end    
  end
  
end