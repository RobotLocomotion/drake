classdef TrajectoryOptimization < NonlinearProgramWConstraintObjects
  %TRAJECTORYOPTIMIZATION Summary of this class goes here
  %   Detailed explanation goes here
  
  properties (Access = public)
    N
    options
    z0
    plant
    h_inds
    x_inds
    u_inds
  end
  
  methods
    % assumes constraints and cost are time-invariant
    % h(i:N-1) - timestep
    % t(i:N) - time
    % x(i:N) - state
    % u(i:N) - control input
    %
    % initial_cost is a NonlinearConstraint eval(x(1))
    % running_cost is a NonlinearConstraint eval([h(i);x(i);u(i)]), summed
    % final_cost is a NonlinearConstraint eval([h(1:end),x])
    % t_init (Nx1) vector of initial times
    % traj_init.x x trajectory
    % traj_init.u u trajectory
    % constraints are each a struct
    %  constraint.mgr is a ConstraintManager
    %  constraint.i is a cell array of time indices
    %   ex1., i = {1, 2, 3} means the constraint is applied
    %   individually to knot points 1, 2, and 3
    %   ex2,. i = {[1 2], [3 4]} means the constraint is applied to knot
    %   points 1 and 2 together (taking the combined state as an argument)
    %   and 3 and 4 together.
    
    function obj = TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,varargin)
      N = length(t_init);
      if ~isempty(varargin)
        options = varargin{1}{end}; %#ok<*PROP>
        constraints = varargin{1}(1:end-1);
      else
        options = struct();
        constraints = {};
      end
      
      if ~isfield(options,'time_option')
        options.time_option = 1;
      end
      
      [num_vars,h_inds,x_inds,u_inds] = TrajectoryOptimization.getVarInfo(plant,N,options);
      
      obj = obj@NonlinearProgramWConstraintObjects(num_vars);
      obj.N = N;
      obj.options = options;
      obj.plant = plant;
      obj.h_inds = h_inds;
      obj.x_inds = x_inds;
      obj.u_inds = u_inds;
      obj.z0 = obj.getInitialVars(t_init,traj_init);
      
      % Construct time constraint
      switch options.time_option
        case 1
          A_time = [ones(1,N-1);[eye(N-2) zeros(N-2,1)] - [zeros(N-2,1) eye(N-2)]];
          time_constraint = LinearConstraint([T_span(1);zeros(N-2,1)],[T_span(2);zeros(N-2,1)],A_time);
          obj = obj.addLinearConstraint(time_constraint,h_inds);
      end
      
      
      
      [dynamic_constraints,dyn_inds] = obj.createDynamicConstraints();
      
      for i=1:length(dynamic_constraints),
        obj = obj.addNonlinearConstraint(dynamic_constraints{i}, dyn_inds{i});
      end
      
      for i=1:length(constraints),
        mgr = constraints{i}.mgr;
        time_index = constraints{i}.i;
        for j=1:length(time_index),
          cstr_inds = [];
          for k=1:length(time_index{j}),
            ind_k = time_index{j}(k);
            cstr_inds = [cstr_inds;x_inds(:,ind_k)]; %#ok<AGROW>
          end
          
          lincon = mgr.getLinearConstraints();
          for k=1:length(lincon),
            obj = obj.addLinearConstraint(lincon{k},cstr_inds);
          end
          
          nlncon = mgr.getNonlinearConstraints();
          for k=1:length(nlncon),
            obj = obj.addNonlinearConstraint(nlncon{k},cstr_inds);
          end
          
          bcon = mgr.getBoundingBoxConstraints();
          for k=1:length(bcon),
            obj = obj.addBoundingBoxConstraint(bcon{k},cstr_inds);
          end
        end
      end
      
      obj = obj.setupCostFunction(initial_cost,running_cost,final_cost);
    end

    
    function [xtraj,utraj,z,F,info] = solveTraj(obj)
      [z,F,info] = obj.solve(obj.z0);
      t = [0; cumsum(z(obj.h_inds))];
      xtraj = PPTrajectory(foh(t,reshape(z(obj.x_inds),[],obj.N)));
      utraj = PPTrajectory(foh(t,reshape(z(obj.u_inds),[],obj.N)));
      
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
      utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
    end
    
    function z0 = getInitialVars(obj,t_init,traj_init)
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(t_init);
      
      for i=1:length(t_init),
        z0(obj.x_inds(:,i)) = traj_init.x.eval(t_init(i));
        z0(obj.u_inds(:,i)) = traj_init.u.eval(t_init(i));
      end
    end
  end
  
  methods(Abstract)    
    obj = setupCostFunction(obj,initial_cost,running_cost,final_cost);
    
    [constraints,dyn_inds] = createDynamicConstraints(obj);
  end
  
  
  methods(Static)
    % Default implementation,
    % Assumes, if time is not fixed, that there are N-1 time steps
    % N corresponding state variables
    % and N-1 corresponding input variables
    % Overwrite to change
    function [num_vars,h_inds,x_inds,u_inds] = getVarInfo(plant,N,options)
      nH = N-1;
      nX = plant.getNumStates();
      nU = plant.getNumInputs();
      
      num_vars = nH + N*(nX+nU);
      h_inds = 1:nH;
      x_inds = reshape(nH + (1:nX*N),nX,N);
      u_inds = reshape(nH + nX*N + (1:nU*N),nU,N);
    end
  end
end

