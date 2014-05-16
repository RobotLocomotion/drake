classdef FEDirtranTrajectoryOptimization < TrajectoryOptimization
  
  properties
  end
  
  methods
    function obj = FEDirtranTrajectoryOptimization(plant,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,varargin)
      obj = obj@TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,varargin);
    end
    
    function [constraints,dyn_inds] = createDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + nU + 1;
      cfun = @(z) constraint_fun(z(1),z(2:nX+1),z(nX+2:2*nX+1),z(2*nX+2:2*nX+nU+1));
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,cfun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i+1)];
        constraints{i} = cnstr;
      end
      
      function [f,df] = constraint_fun(h,x0,x1,u)
        [xdot,dxdot] = geval(@(t,x,u) dynamics(obj.plant,t,x,u),0,x0,u);
%         [xdot,dxdot] = obj.plant.dynamics(0,x0,u);
        f = x1 - x0 - h*xdot;
        df = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
      end
    end
    
    function obj = setupCostFunction(obj,initial_cost,running_cost,final_cost)
      if ~isempty(initial_cost)
        obj = obj.addCost(initial_cost,obj.x_inds(:,1));
      end
      
      if ~isempty(running_cost)
        for i=2:obj.N,
          h_ind = obj.h_inds(i-1);
          x_ind = obj.x_inds(:,i-1);
          u_ind = obj.u_inds(:,i);
          
          obj = obj.addCost(running_cost,[h_ind;x_ind;u_ind]);
        end        
      end
      
      h_ind = obj.h_inds;
      x_ind = obj.x_inds(:,end);
      
      if ~isempty(final_cost)
        obj = obj.addCost(final_cost,[h_ind;x_ind]);
      end
    end
    
  end
end