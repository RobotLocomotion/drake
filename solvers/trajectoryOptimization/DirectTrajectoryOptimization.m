classdef DirectTrajectoryOptimization < NonlinearProgramWConstraintObjects
  %DIRECTTRAJECTORYOPTIMIZATION An abstract class for direct method approaches to
  % trajectory optimization.
  %
  % Generally considers cost functions of the form:
  % e(x0) + int(f(x(t),u(t)) + g(T,xf)
  %    
  % Subclasses must implement the two abstract methods:
  %  obj = addRunningCost(obj,running_cost);
  %     where running_cost is a NonlinearConstraint f(h,x,u)
  % and
  %  obj = addDynamicConstraints(obj);
  %   which add the constraints to enforce \dot x = f(x,u)
  %
  % This class assumes that there are a fixed number (N) time steps, and
  % that the trajectory is discreteized into timesteps h (N-1), state x
  % (N), and control input u (N)
  %
  % To maintain nominal sparsity in the optimization programs, this
  % implementation assumes that all constraints and costs are
  % time-invariant.
  
  properties (SetAccess = protected)
    N       % number of timesteps
    options % options, yup
    plant   % the plant
    h_inds  % (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
    x_inds  % N x n indices for state
    u_inds  % N x m indices for time
    dynamic_constraints = {};
    constraints = {};
  end
  
  methods
    % function obj =
    % DirectTrajectoryOptimization(plant,initial_cost,running_cost,final_cost,...
    % t_init,traj_init,T_span,constraints, options)
    % Trajectory optimization constructor
    % @param plant
    % @param N the number of time samples
    % @param T_span  The lower and upper bounds on total time for the trajectory [lb ub]
    % @param options (optional)
    %        options.time_option {1: all time steps are constant, 2: all
    %        time steps are independent}
    function obj = DirectTrajectoryOptimization(plant,N,T_span,options)
      %#ok<*PROP>
      
      if nargin < 4
        options = struct();
      end
      
      if ~isfield(options,'time_option')
        options.time_option = 1;
      end
      
      %todo: replace getVarInfo with setupVarInfo
      % initialize with 0 variables and then add them
      
      obj = obj@NonlinearProgramWConstraintObjects(0);
      obj.options = options;
      obj.plant = plant;
      
      obj = obj.setupVariables(N);
      
      % Construct total time linear constraint
      switch options.time_option
        case 1 % all timesteps are constant
          A_time = [ones(1,N-1);[eye(N-2) zeros(N-2,1)] - [zeros(N-2,1) eye(N-2)]];
          time_constraint = LinearConstraint([T_span(1);zeros(N-2,1)],[T_span(2);zeros(N-2,1)],A_time);
          obj = obj.addLinearConstraint(time_constraint,obj.h_inds);
        case 2 % all timesteps independent
          A_time = ones(1,N-1);
          time_constraint = LinearConstraint(T_span(1),T_span(2),A_time);
          obj = obj.addLinearConstraint(time_constraint,obj.h_inds);
      end
      
      
      % create constraints for dynamics and add them
      obj = obj.addDynamicConstraints();
      
      % add control inputs as bounding box constraints
      control_limit = BoundingBoxConstraint(repmat(plant.umin,N,1),repmat(plant.umax,N,1));
      obj = obj.addBoundingBoxConstraint(control_limit,obj.u_inds(:));
      
      % add joint limits as bounding box constraints
      [joint_lb,joint_ub] = plant.getJointLimits();
      joint_limit = BoundingBoxConstraint(repmat(joint_lb,N,1),repmat(joint_ub,N,1));
      obj = obj.addBoundingBoxConstraint(joint_limit,reshape(obj.x_inds(1:plant.getNumPositions,:),[],1));
    end
    
    function obj = addManagedStateConstraint(obj,constraint,time_index)
      % Add a linear constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a ConstraintManager
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~iscell(time_index)
        time_index = {time_index};
      end
      for j=1:length(time_index),
        cstr_inds = mat2cell(obj.x_inds(:,time_index{j}),size(obj.x_inds,1),ones(1,length(time_index{j})));
        
        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cstr_inds;
        obj.constraints{end}.time_index = time_index;
        
        obj = obj.addManagedConstraints(constraint,cstr_inds);
      end
    end
    
    function obj = addLinearStateConstraint(obj,constraint,time_index)
      % Add a linear constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a LinearConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      obj = obj.addManagedStateConstraint(ConstraintManager(constraint),time_index);
    end
    
    function obj = addNonlinearStateConstraint(obj,constraint,time_index)
      % Add a nonlinear constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a NonlinearConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      obj = obj.addManagedStateConstraint(ConstraintManager([],constraint),time_index);
    end
    
    function obj = addBoundingBoxStateConstraint(obj,constraint,time_index)
      % Add a boundingbox constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a BoundingBoxConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      %
      obj = obj.addManagedStateConstraint(ConstraintManager([],[],constraint),time_index);
    end
    
    function obj = addStateConstraint(obj,constraint,time_index)
      % Add a constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a NonlinearConstraint or ConstraintManager object
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      %
      
      if isa(constraint,'BoundingBoxConstraint')
        obj = addBoundingBoxStateConstraint(obj,constraint,time_index);
      elseif isa(constraint,'LinearConstraint')
        obj = addLinearStateConstraint(obj,constraint,time_index);
      elseif isa(constrain,'NonlinearConstraint')
        obj = addNonlinearStateConstraint(obj,constraint,time_index);
      elseif isa(constraint,'ConstraintManager')
        obj = addManagedStateConstraint(obj,ConstraintManager,time_index);
      else
        error('Drake:DirectTrajectoryOptimization:UnsupportedStateConstraint','Unsupported state constraint type');
      end
    end
    
    % Solve the nonlinear program and return resulting trajectory
    function [xtraj,utraj,z,F,info] = solveTraj(obj,t_init,traj_init)
      z0 = obj.getInitialVars(t_init,traj_init);
      [z,F,info] = obj.solve(z0);
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
      if length(t_init) ~= obj.N
        error('The initial sample times must have the same length as property N')
      end
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(t_init);
      
      for i=1:length(t_init),
        z0(obj.u_inds(:,i)) = traj_init.u.eval(t_init(i));
        
        if isfield(traj_init,'x')
          z0(obj.x_inds(:,i)) = traj_init.x.eval(t_init(i));
        else
          %simulate
          sys_ol = cascade(traj_init.u,obj.plant);
          [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)]);
          z0(obj.x_inds(:,i)) = x_sim.x.eval(t_init(i));
        end
      end
    end
    
    function obj = setupVariables(obj, N)
      % Default implementation,
      % Assumes, if time is not fixed, that there are N-1 time steps
      % N corresponding state variables
      % and N-1 corresponding input variables
      % Overwrite to change
      %
      % Generates num_vars total number of decision variables
      %   h_inds (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
      %   x_inds N x n indices for state
      %   u_inds N x m indices for time
      %
      % @param N number of knot points
      nH = N-1;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      num_vars = nH + N*(nX+nU);
      obj.h_inds = (1:nH)';
      obj.x_inds = reshape(nH + (1:nX*N),nX,N);
      obj.u_inds = reshape(nH + nX*N + (1:nU*N),nU,N);
      
      obj.N = N;
      
      obj = obj.addDecisionVariable(num_vars);
    end
    
    function obj = addInitialCost(obj,initial_cost)
      % Adds a cost to the initial state f(x0)
      obj = obj.addCost(initial_cost,obj.x_inds(:,1));
    end
    
    function obj = addFinalCost(obj,final_cost)
      % adds a cost to the final state and total time f(T,xf);
      obj = obj.addCost(final_cost,{obj.h_inds;obj.x_inds(:,end)});
    end
  end
  
  methods(Abstract)
    % Adds an integrated cost, f(h,x,u) to all time steps, which is
    % numerical implementation specific (thus abstract)
    obj = addRunningCost(obj,running_cost);
    
    obj = addDynamicConstraints(obj);
  end
end

