classdef MidpointTrajectoryOptimization < TrajectoryOptimization
  % Midpoint method integration
  %  dynamics constraints are: x(k+1) = x(k) + h(k)*f(.5*x(k)+.5*x(k+1),.5*u(k)+.5*u(k+1))
  %  integrated cost is sum of g(h(k),.5*x(k)+.5*x(k+1),.5*u(k)+.5*u(k+1))
  
  properties
  end
  
  methods
    function obj = MidpointTrajectoryOptimization(plant,initial_cost,running_cost,final_cost,N,T_span,varargin)
      obj = obj@TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,N,T_span,varargin{:});
    end
    
    function [constraints,dyn_inds] = createDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + 2*nU + 1;
      cfun = @(z) constraint_fun(z(1),z(2:nX+1),z(nX+2:2*nX+1),z(2*nX+2:2*nX+nU+1),z(2*nX+nU+2:2*nX+2*nU+1));
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,cfun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)];
        constraints{i} = cnstr;
      end
      
      function [f,df] = constraint_fun(h,x0,x1,u0,u1)
%         [xdot,dxdot] = geval(@(t,x,u) dynamics(obj.plant,t,x,u),0,.5*(x0+x1),.5*(u0+u1));
        [xdot,dxdot] = obj.plant.dynamics(0,.5*(x0+x1),.5*(u0+u1));
        f = x1 - x0 - h*xdot;
        df = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) -.5*h*dxdot(:,nX+2:end) -.5*h*dxdot(:,nX+2:end)];
      end
    end
    
    function obj = setupCostFunction(obj,initial_cost,running_cost,final_cost)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      if ~isempty(initial_cost)
        obj = obj.addCost(initial_cost,obj.x_inds(:,1));
      end
      
      running_handle = running_cost.eval_handle;
      running_handle_i = @(z) running_fun(running_handle,z(1),z(2:1+nX), z(2+nX:1+2*nX),z(2+2*nX:1+2*nX+nU),z(2+2*nX+nU:1+2*nX+2*nU)); 
      running_cost_i = NonlinearConstraint(running_cost.lb,running_cost.ub,1+2*nX+2*nU,running_handle_i);
      
      if ~isempty(running_cost)
        for i=1:obj.N-1,
          h_ind = obj.h_inds(i);
          x_ind = [obj.x_inds(:,i);obj.x_inds(:,i+1)];
          u_ind = [obj.u_inds(:,i);obj.u_inds(:,i+1)];
          
          obj = obj.addCost(running_cost_i,[h_ind;x_ind;u_ind]);
        end        
      end
      
      h_ind = obj.h_inds;
      x_ind = obj.x_inds(:,end);
      
      if ~isempty(final_cost)
        obj = obj.addCost(final_cost,[h_ind;x_ind]);
      end
      
      function [f,df] = running_fun(cost_handle,h,x0,x1,u0,u1)
        [f,dg] = cost_handle([h;.5*(x0+x1);.5*(u0+u1)]);
        
        df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU)];
      end
    end
    
  end
end