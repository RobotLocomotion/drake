classdef TrajectoryOptimization < NonlinearProgramWConstraintObjects
  %TRAJECTORYOPTIMIZATION An abstract class for direct method approaches to
  % trajectory optimization.
  % 
  % Generally considers cost functions of the form:
  % e(x0) + int(f(x(t),u(t)) + g(T,xf)
  %
  % Subclasses must implement the two abstract methods:
  %  obj = setupCostFunction(obj,initial_cost,running_cost,final_cost);
  % and  
  %  [constraints,dyn_inds] = createDynamicConstraints(obj);
  % which each determine how the dynamic constraints are evaluated and how
  % the cost function is integrated.
  %
  % This class assumes that there are a fixed number (N) time steps, and
  % that the trajectory is discreteized into timesteps h (N-1), state x
  % (N), and control input u (N)
  %
  % To maintain nominal sparsity in the optimization programs, this
  % implementation assumes that all constraints and costs are
  % time-invariant.
  %
  % See Hargraves87 and Enright91
  
  properties (Access = public)
    N       % number of timesteps
    options % options, yup
    z0      % initial optimization paramters, extracted from trajectories
    plant   % the plant
    h_inds  % (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
    x_inds  % N x n indices for state
    u_inds  % N x m indices for time
  end
  
  methods
    % function obj =
    % TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,...
    % t_init,traj_init,T_span,constraints, options)
    % Trajectory optimization constructor 
    
    %
    % initial_cost is a NonlinearConstraint to be evaluated at x(1)
    % running_cost is a NonlinearConstraint to be evaluated at a given h,x,u
    % final_cost is a NonlinearConstraint to be evaluated at eval(T,x])
    % t_init (Nx1) vector of initial times
    % traj_init is a struct, where
    %  traj_init.x x trajectory
    %  traj_init.u u trajectory
    % constraints is a cell-array of structs
    %  constraint.mgr is a ConstraintManager
    %  constraint.i is a cell array of time indices
    %   ex1., i = {1, 2, 3} means the constraint is applied
    %   individually to knot points 1, 2, and 3
    %   ex2,. i = {[1 2], [3 4]} means the constraint is applied to knot
    %   points 1 and 2 together (taking the combined state as an argument)
    %   and 3 and 4 together.
    % 
    
    function obj = TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,constraints,options)
      if nargin < 8
        constraints = {};
      end
      
      if nargin < 9
        options = struct();
      end
      
      N = length(t_init);
      
      if ~isfield(options,'time_option')
        options.time_option = 1;
      end
      
      [num_vars,h_inds,x_inds,u_inds] = TrajectoryOptimization.getVarInfo(plant,N,options); %#ok<*PROP>
      
      obj = obj@NonlinearProgramWConstraintObjects(num_vars);
      obj.N = N;
      obj.options = options;
      obj.plant = plant;
      obj.h_inds = h_inds;
      obj.x_inds = x_inds;
      obj.u_inds = u_inds;
      obj.z0 = obj.getInitialVars(t_init,traj_init);
      
      % Construct total time linear constraint
      switch options.time_option
        case 1 % all timesteps are constant
          A_time = [ones(1,N-1);[eye(N-2) zeros(N-2,1)] - [zeros(N-2,1) eye(N-2)]];
          time_constraint = LinearConstraint([T_span(1);zeros(N-2,1)],[T_span(2);zeros(N-2,1)],A_time);
          obj = obj.addLinearConstraint(time_constraint,h_inds);
        case 2 % all timesteps independent
          A_time = ones(1,N-1);
          time_constraint = LinearConstraint(T_span(1),T_span(2),A_time);
          obj = obj.addLinearConstraint(time_constraint,h_inds);
      end
      
      
      % create constraints for dynamics and add them
      [dynamic_constraints,dyn_inds] = obj.createDynamicConstraints();
      
      for i=1:length(dynamic_constraints),
        obj = obj.addNonlinearConstraint(dynamic_constraints{i}, dyn_inds{i});
      end
      
      % add control inputs as bounding box constraints
      control_limit = BoundingBoxConstraint(repmat(plant.umin,N,1),repmat(plant.umax,N,1));
      obj = obj.addBoundingBoxConstraint(control_limit,obj.u_inds(:));
      
      % loop over additional constraints
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
      
      % setup the cost function
      obj = obj.setupCostFunction(initial_cost,running_cost,final_cost);
    end
        
    % Solve the nonlinear program and return resulting trajectory
    function [xtraj,utraj,z,F,info] = solveTraj(obj)
      [z,F,info] = obj.solve(obj.z0);
      t = [0; cumsum(z(obj.h_inds))];
      xtraj = PPTrajectory(foh(t,reshape(z(obj.x_inds),[],obj.N)));
      utraj = PPTrajectory(foh(t,reshape(z(obj.u_inds),[],obj.N)));
      
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
      utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
    end
    
    % evaluates the initial trajectories at the sampled times and
    % constructs the nominal z0. Overwrite to implement in a different
    % manner
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
    % Add the initial, running, and final costs to the program
    % this is abstract since the running cost involves numerical
    % integration, which will vary depending on the particular direct
    % method used
    obj = setupCostFunction(obj,initial_cost,running_cost,final_cost);
    
    % Create a cell array of constraints and a cell array of indices that
    % represent the dynamics constraints
    [constraints,dyn_inds] = createDynamicConstraints(obj);
  end
  
  
  methods(Static)
    % Default implementation,
    % Assumes, if time is not fixed, that there are N-1 time steps
    % N corresponding state variables
    % and N-1 corresponding input variables
    % Overwrite to change
    %
    % @param plant
    % @param N number of knot points
    % @param options
    %
    % @return num_vars total number of decision variables
    % @return h_inds (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
    % @return x_inds N x n indices for state
    % @return u_inds N x m indices for time
    function [num_vars,h_inds,x_inds,u_inds] = getVarInfo(plant,N,options)
      nH = N-1;
      nX = plant.getNumStates();
      nU = plant.getNumInputs();
      
      num_vars = nH + N*(nX+nU);
      h_inds = (1:nH)';
      x_inds = reshape(nH + (1:nX*N),nX,N);
      u_inds = reshape(nH + nX*N + (1:nU*N),nU,N);
    end
  end
end

