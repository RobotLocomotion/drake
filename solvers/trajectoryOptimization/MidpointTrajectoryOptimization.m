classdef MidpointTrajectoryOptimization < DirectTrajectoryOptimization
  % Midpoint method integration
  %  dynamics constraints are: x(k+1) = x(k) + h(k)*f(.5*x(k)+.5*x(k+1),.5*u(k)+.5*u(k+1))
  %  integrated cost is sum of g(h(k),.5*x(k)+.5*x(k+1),.5*u(k)+.5*u(k+1))
  
  properties
  end
  
  methods
    function obj = MidpointTrajectoryOptimization(plant,N,T_span,varargin)
      obj = obj@DirectTrajectoryOptimization(plant,N,T_span,varargin{:});
    end
    
    function obj = addDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      n_vars = 2*nX + 2*nU + 1;
      cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.midpoint_constraint_fun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)};
        constraints{i} = cnstr;
        
        obj = obj.addConstraint(constraints{i}, dyn_inds{i});
      end
    end
        
    function obj = addTIRunningCost(obj,running_cost_function)
      % Add a time invariant running cost by integrating the running cost
      % over time
      % @param running_cost_function a function handle cost = f(h, x,u)
  
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      running_cost = FunctionHandleObjective(1+2*nX+2*nU,...
        @(h,x0,x1,u0,u1) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0,u1));    
      for i=1:obj.N-1,
        obj = obj.addCost(running_cost,{obj.h_inds(i);obj.x_inds(:,i);...
          obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)});
      end
    end
    
    function obj = addTVRunningCost(obj,running_cost_function)
      % Add a time varying running cost by integrating the running cost
      % over time
      % @param running_cost_function a function handle cost = f(h,t,x,u)
  
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      running_cost = FunctionHandleObjective(1+2*nX+2*nU,...
        @(h,x0,x1,u0,u1) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0,u1));    
      for i=1:obj.N-1,
        obj = obj.addCost(running_cost,{obj.h_inds(i);obj.x_inds(:,i);...
          obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)});
      end
    end
  end
  
  methods (Access=protected)    
    function [f,df] = midpoint_constraint_fun(obj,h,x0,x1,u0,u1)
      nX = obj.plant.getNumStates();
      [xdot,dxdot] = obj.plant.dynamics(0,.5*(x0+x1),.5*(u0+u1));
      f = x1 - x0 - h*xdot;
      df = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) ...
        -.5*h*dxdot(:,nX+2:end) -.5*h*dxdot(:,nX+2:end)];
    end
    
    function [f,df] = midpoint_running_fun(obj,running_handle,h,x0,x1,u0,u1)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      [f,dg] = running_handle(h,.5*(x0+x1),.5*(u0+u1));
      
      df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU)];
    end
  end
end