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
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,@constraint_fun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)};
        constraints{i} = cnstr;
        
        obj = obj.addNonlinearConstraint(constraints{i}, dyn_inds{i});
      end
      
      function [f,df] = constraint_fun(h,x0,x1,u0,u1)
%         [xdot,dxdot] = geval(@(t,x,u) dynamics(obj.plant,t,x,u),0,.5*(x0+x1),.5*(u0+u1));
        [xdot,dxdot] = obj.plant.dynamics(0,.5*(x0+x1),.5*(u0+u1));
        f = x1 - x0 - h*xdot;
        df = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) -.5*h*dxdot(:,nX+2:end) -.5*h*dxdot(:,nX+2:end)];
      end
    end
    
    function obj = addRunningCost(obj,running_cost)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      running_handle = running_cost.eval_handle;
      running_cost_i = NonlinearConstraint(running_cost.lb,running_cost.ub,1+2*nX+2*nU,@running_fun);
      
      if ~isempty(running_cost)
        for i=1:obj.N-1,          
          obj = obj.addCost(running_cost_i,{obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)});
        end        
      end
      
      h_ind = obj.h_inds;
      x_ind = obj.x_inds(:,end);
      
      function [f,df] = running_fun(h,x0,x1,u0,u1)
        [f,dg] = running_handle(h,.5*(x0+x1),.5*(u0+u1));
        
        df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU)];
      end
    end
    
  end
end