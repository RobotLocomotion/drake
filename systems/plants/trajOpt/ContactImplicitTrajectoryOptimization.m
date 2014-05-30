classdef ContactImplicitTrajectoryOptimization < TrajectoryOptimization
  % phi, lambda
  properties
    l_inds
  end
  
  methods
    function obj = ContactImplicitTrajectoryOptimization(plant,initial_cost,running_cost,final_cost,N,T_span,varargin)
      obj = obj@TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,N,T_span,varargin{:});
    end
    
    function obj = addDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + nU + 1;
      dynfun = @(z) dynamics_constraint_fun(z(1),z(2:nX+1),z(nX+2:2*nX+1),z(2*nX+2:2*nX+nU+1));
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,dynfun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)];
        constraints{i} = cnstr;
      end
      
      function [f,df] = dynamics_constraint_fun(h,x0,x1,u)
        [xdot,dxdot] = geval(@(t,x,u) dynamics(obj.plant,t,x,u),0,x0,u);
%         [xdot,dxdot] = obj.plant.dynamics(0,x0,u);
        f = x1 - x0 - h*xdot;
        df = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
      end
    end
    
    function obj = setupVariables(obj,N)
      obj = setupVariables@TrajectoryOptimization(N);
    end
    
    % evaluates the initial trajectories at the sampled times and
    % constructs the nominal z0. Overwrite to implement in a different
    % manner
    function z0 = getInitialVars(obj,t_init,traj_init)
      if length(t_init) ~= obj.N
        error('The initial sample times must have the same length as property N')
      end
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(t_init);
      
      for i=1:length(t_init),
        z0(obj.x_inds(:,i)) = traj_init.x.eval(t_init(i));
        z0(obj.u_inds(:,i)) = traj_init.u.eval(t_init(i));
      end
    end
    
    function obj = setupCostFunction(obj,initial_cost,running_cost,final_cost)
      if ~isempty(initial_cost)
        obj = obj.addCost(initial_cost,obj.x_inds(:,1));
      end
      
      if ~isempty(running_cost)
        for i=1:obj.N-1,
          h_ind = obj.h_inds(i);
          x_ind = obj.x_inds(:,i);
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