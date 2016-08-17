classdef DirectTrajectoryOptimization < NonlinearProgram
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
  %   which add the constraints to enforce xdot = f(x,u)
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
    x_inds  % n x N indices for state
    u_inds  % m x N indices for time
    dynamic_constraints = {};
    constraints = {};
  end

  methods
    function obj = DirectTrajectoryOptimization(plant,N,durations,options)
    % function obj =
    % DirectTrajectoryOptimization(plant,initial_cost,running_cost,final_cost,...
    % t_init,traj_init,T_span,constraints, options)
    % Trajectory optimization constructor
    % @param plant
    % @param N the number of time samples
    % @param duration  The lower and upper bounds on total time for the trajectory [lb ub]
    % @param options (optional)
    %        options.time_option {1: all time steps are constant, 2: all
    %        time steps are independent}

    %#ok<*PROP>

      if isscalar(durations), durations=[durations,durations]; end
      if nargin < 4
        options = struct();
      end

      if ~isfield(options,'time_option')
        options.time_option = 1;
      end
      if ~plant.isTI
        error('Drake:DirectTrajectoryOptimization:UnsupportedPlant','Only time-invariant plants are currently supported');
      end

      %todo: replace getVarInfo with setupVarInfo
      % initialize with 0 variables and then add them

      obj = obj@NonlinearProgram(0);
      obj.options = options;
      obj.plant = plant;

      obj = obj.setupVariables(N);

      % Construct total time linear constraint
      switch options.time_option
        case 1 % all timesteps are constant
          A_time = [ones(1,N-1);[eye(N-2) zeros(N-2,1)] - [zeros(N-2,1) eye(N-2)]];
          time_constraint = LinearConstraint([durations(1);zeros(N-2,1)],[durations(2);zeros(N-2,1)],A_time);
          obj = obj.addConstraint(time_constraint,obj.h_inds);
        case 2 % all timesteps independent
          A_time = ones(1,N-1);
          time_constraint = LinearConstraint(durations(1),durations(2),A_time);
          obj = obj.addConstraint(time_constraint,obj.h_inds);
      end

      % Ensure that all h values are non-negative
      obj = obj.addConstraint(BoundingBoxConstraint(zeros(N-1,1),inf(N-1,1)),obj.h_inds);

      % create constraints for dynamics and add them
      obj = obj.addDynamicConstraints();

      % add control inputs as bounding box constraints
      if any(~isinf(plant.umin)) || any(~isinf(plant.umax))
        control_limit = BoundingBoxConstraint(repmat(plant.umin,N,1),repmat(plant.umax,N,1));
        obj = obj.addConstraint(control_limit,obj.u_inds(:));
      end

    end
    
    function N = getN(obj)
      N = obj.N;
    end

    function x_inds = getXinds(obj)
      x_inds = obj.x_inds;
    end

    function h_inds = getHinds(obj)
      h_inds = obj.h_inds;
    end

    function obj = addInputConstraint(obj,constraint,time_index)
      % Add constraint (or composite constraint) that is a function of the
      % input at the specified time or times.
      % @param constraint  a Constraint or CompositeConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3. For convenience, [1,2,3] is also
      %   interpreted as {1,2,3}.
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~iscell(time_index)
        time_index = num2cell(reshape(time_index,1,[]));
      end
      for j=1:length(time_index),
        cstr_inds = mat2cell(obj.u_inds(:,time_index{j}),size(obj.u_inds,1),ones(1,length(time_index{j})));

        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cstr_inds;
        obj.constraints{end}.time_index = time_index;

        obj = obj.addConstraint(constraint,cstr_inds);
      end
    end

    function obj = addStateConstraint(obj,constraint,time_index,x_indices)
      % Add constraint (or composite constraint) that is a function of the
      % state at the specified time or times.
      % @param constraint  a CompositeConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3. For convenience, [1,2,3] is also
      %   interpreted as {1,2,3}.
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      % @param x_index optional subset of the state vector x which this
      % constraint depends upon @default 1:num_x
      if ~iscell(time_index)
        % then use { time_index(1), time_index(2), ... } ,
        % aka independent constraints for each time
        time_index = num2cell(reshape(time_index,1,[]));
      end
      if nargin<4, x_indices = 1:size(obj.x_inds,1); end

      for j=1:length(time_index),
        cstr_inds = mat2cell(obj.x_inds(x_indices,time_index{j}),numel(x_indices),ones(1,length(time_index{j})));

        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cstr_inds;
        obj.constraints{end}.time_index = time_index;

        obj = obj.addConstraint(constraint,cstr_inds);
      end
    end

    function obj = addTrajectoryDisplayFunction(obj,display_fun)
      % add a dispay function that gets called on every iteration of the
      % algorithm
      % @param display_fun a function handle of the form displayFun(t,x,u)
      %       where t is a 1-by-N, x is n-by-N and u is m-by-N
      
      obj = addDisplayFunction(obj,@(z) display_fun(z(obj.h_inds),z(obj.x_inds),z(obj.u_inds)));
    end
    
    function [xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTraj(obj,t_init,traj_init)
      % Solve the nonlinear program and return resulting trajectory
      % @param t_init initial timespan for solution.  can be a vector of
      % length obj.N specifying the times of each segment, or a scalar
      % indicating the final time.
      % @param traj_init (optional) a structure containing Trajectory
      % objects specifying the initial guess for the system inputs
      %    traj_init.u , traj_init.x, ...
      % @default small random numbers

      if nargin<3, traj_init = struct(); end

      z0 = obj.getInitialVars(t_init,traj_init);
      [z,F,info,infeasible_constraint_name] = obj.solve(z0);
      xtraj = reconstructStateTrajectory(obj,z);
      if nargout>1, utraj = reconstructInputTrajectory(obj,z); end
    end

    function z0 = getInitialVars(obj,t_init,traj_init)
    % evaluates the initial trajectories at the sampled times and
    % constructs the nominal z0. Overwrite to implement in a different
    % manner
      if isscalar(t_init)
        t_init = linspace(0,t_init,obj.N);
      elseif length(t_init) ~= obj.N
        error('The initial sample times must have the same length as property N')
      end
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(t_init);

      if nargin<3, traj_init = struct(); end

      nU = getNumInputs(obj.plant);
      if isfield(traj_init,'u')
        z0(obj.u_inds) = traj_init.u.eval(t_init);
      else
        z0(obj.u_inds) = 0.01*randn(nU,obj.N);
      end

      if isfield(traj_init,'x')
        z0(obj.x_inds) = traj_init.x.eval(t_init);
      else
        if nU>0
          if ~isfield(traj_init,'u')
            traj_init.u = setOutputFrame(PPTrajectory(foh(t_init,reshape(z0(obj.u_inds),nU,obj.N))),getInputFrame(obj.plant));
          end
          
          % todo: if x0 and xf are equality constrained, then initialize with
          % a straight line from x0 to xf (this was the previous behavior)
          
          %simulate
          sys_ol = cascade(traj_init.u,obj.plant);
        else
          sys_ol = obj.plant;
        end
        
        if ~isfield(traj_init,'x0')
          [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)]);
        else
          [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)],traj_init.x0);
        end
        
        z0(obj.x_inds) = x_sim.eval(t_init);
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
      x_names = cell(num_vars,1);
      for i = 1:N
        if(i<N)
          x_names{i} = sprintf('h[%d]',i);
        end
        for j = 1:nX
          x_names{nH+(i-1)*nX+j}=sprintf('x%d[%d]',j,i);
        end
        for j = 1:nU
          x_names{nH+nX*N+(i-1)*nU+j} = sprintf('u%d[%d]',j,i);
        end
      end

      obj = obj.addDecisionVariable(num_vars,x_names);
    end

    function obj = addInitialCost(obj,initial_cost)
      % Adds a cost to the initial state f(x0)
      obj = obj.addCost(initial_cost,obj.x_inds(:,1));
    end

    function obj = addFinalCost(obj,final_cost_function)
      % adds a cost to the final state and total time
      % @param final_cost_function a function handle f(T,xf)
      nX = obj.plant.getNumStates();
      nH = obj.N-1;
      cost = FunctionHandleObjective(nH+nX,@(h,x) obj.final_cost(final_cost_function,h,x));
      obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end)});
    end

    function utraj = reconstructInputTrajectory(obj,z)
      % default behavior is to use first order holds, but this can be
      % re-implemented by a subclass.
      t = [0; cumsum(z(obj.h_inds))];

      if size(obj.u_inds,1)>0
        u = reshape(z(obj.u_inds),[],obj.N);
        utraj = PPTrajectory(foh(t,u));
        utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
      else
        utraj=[];
      end
    end

    function xtraj = reconstructStateTrajectory(obj,z)
      % default behavior is to use first order holds, but this can be
      % re-implemented by a subclass.
      t = [0; cumsum(z(obj.h_inds))];

      x = reshape(z(obj.x_inds),[],obj.N);
      xtraj = PPTrajectory(foh(t,x));
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
    end

    function u0 = extractFirstInput(obj,z)
      % When using trajectory optimization a part of a model-predictive
      % control system, we often only need to extract u(0).  This method
      % does exactly that.

      u0 = z(obj.u_inds(:,1));
    end
  end

  methods(Access=protected)
    function [f,df] = final_cost(obj,final_cost_function,h,x)
      T = sum(h);
      [f,dg] = final_cost_function(T,x);
      df = [repmat(dg(:,1),1,length(h)) dg(:,2:end)];
    end
  end

  methods(Abstract)
    % Adds an integrated cost to all time steps, which is
    % numerical implementation specific (thus abstract)
    % this cost is assumed to be time-invariant
    % @param running_cost_function a function handle
    obj = addRunningCost(obj,running_cost_function);


    obj = addDynamicConstraints(obj);
  end
end
