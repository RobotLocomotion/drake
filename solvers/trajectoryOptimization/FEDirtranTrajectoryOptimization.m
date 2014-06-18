classdef FEDirtranTrajectoryOptimization < DirectTrajectoryOptimization
  % Forward-Euler integration
  %  dynamics constraints are: x(k+1) = x(k) + h(k)*f(x(k),u(k))
  %  integrated cost is sum of g(h(k),x(k),u(k))
  properties
  end
  
  methods
    function obj = FEDirtranTrajectoryOptimization(plant,N,T_span,varargin)
      obj = obj@DirectTrajectoryOptimization(plant,N,T_span,varargin{:});
    end
    
    function obj = addDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);      
      
      n_vars = 2*nX + nU + 1;
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,@constraint_fun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
        constraints{i} = cnstr;
        
        obj = obj.addNonlinearConstraint(constraints{i}, dyn_inds{i});
      end
      
      function [f,df] = constraint_fun(h,x0,x1,u)
        [xdot,dxdot] = obj.plant.dynamics(0,x0,u);
        f = x1 - x0 - h*xdot;
        df = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
      end
    end
    
    function obj = addRunningCost(obj,running_cost)
      for i=1:obj.N-1,
        h_ind = obj.h_inds(i);
        x_ind = obj.x_inds(:,i);
        u_ind = obj.u_inds(:,i);
        
        obj = obj.addCost(running_cost,{h_ind;x_ind;u_ind});
      end
    end
    
  end
end