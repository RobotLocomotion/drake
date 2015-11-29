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
    
    function runConvexDirtran
      % Solve the min-time problem as a bisection search of 
      % linear programs
      
      x0 = [-2; -2];
      xf = zeros(2,1);
      dt = .1;
      
      A = [ 0, 1; 0 0]; B = [0 ; 1];
      % x[n+1] = Ax[n] + Bu[n]
      Ad = eye(2)+dt*A; Bd = dt*B;
      % Ad = expm(A*dt); Bd = ... % would be slightly better...
      discrete_time_system = LinearSystem([],[],Ad,Bd,eye(2),[]);
      
      warning('off','optim:quadprog:NullHessian');  % I know I'm using QP to solve an LP...
      
      for N = 50:100 % increase N until i find a feasible solution
        % note: bisection search would be a lot more efficient, but it's in
        % the noise here...
        
        % constructs a QP with constraints x[n+1] = Ad*x[n] + Bd*u[n] set up
        [prog,x_inds,u_inds] = dirtranModelPredictiveControl(discrete_time_system,N);

        % add initial value constraint:
        prog = addLinearConstraint(prog,LinearConstraint(x0,x0,eye(2)),x_inds(:,1));
        
        % add final value constraint:
        prog = addLinearConstraint(prog,LinearConstraint(xf,xf,eye(2)),x_inds(:,end));
        
        % add input limit constraints
        prog = addBoundingBoxConstraint(prog,BoundingBoxConstraint(-1*ones(N,1),1*ones(N,1)),u_inds);

        prog = prog.setSolver('quadprog');
        [x,objval,exitflag]=prog.solve();
        if (exitflag>0)
          N
          plot(x(x_inds(1,:)),x(x_inds(2,:)),'b',x(x_inds(1,:)),x(x_inds(2,:)),'b.','LineWidth',2,'MarkerSize',15);
          u = x(u_inds)
          return
        end
      end
      
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