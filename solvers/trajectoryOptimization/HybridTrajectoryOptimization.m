classdef HybridTrajectoryOptimization < NonlinearProgram
  % HybridTrajectoryOptimization
  % 
  % Trajectory optimization class for hybrid models
  %  Works by constructing a series of trajectory optimization programs for
  %  each mode in the mode sequence. The trajectories of each mode are then
  %  constrained so that (1) The transitions between them align
  %  appropriately and (2) Guard functions are triggered at the transitions
  %  and (3) Guard functions are never violated
  %
  %  After construction, constraints can be added to the hybrid
  %  optimization program directly, or to the individual mode
  %  optimizations. Before running, it is recommended to call the compile()
  %  function, which extracts all constraints from the mode optimization
  %  programs and adds them to the main program. compile() should only be
  %  called ONCE!!
  %
  % Basic outline:
  %  traj_opt = HybridTrajectoryOptimizatino(...);
  %  traj_opt = traj_opt.addConstraint(...)
  %  traj_opt = traj_opt.addModeConstraint(...)
  %  ...
  %  traj_opt = traj_opt.compile()
  %  traj_opt.solveTraj()
  properties %(SetAccess=protected)
    N
    M
    mode_sequence
    mode_opt
    mixed_constraint_args
    var_offset
    shared_data_offset
    plant
    is_compiled = false;
  end
  
  methods
    function obj = HybridTrajectoryOptimization(traj_opt,plant,mode_sequence,N,duration,options)
      % @param traj_opt A reference to the appropriate
      % DirectTrajectoryOptimization class (function handle to the
      % constructor)
      % @param HybridDrakeSystem plant
      % @param mode_sequence a mx1 vector of mode indices
      % @param N a mx1 vector of timesteps per mode
      % @param duration a mx1 cell-array of durations
      % @param options:
      %    periodic - enforces that the first state/control of the first mode
      %    equals the last state/control of the last mode
      %    u_const_across_transitions - control input must be constant at
      %    transition times
      if nargin < 6
        options = struct();
      end
      if ~isfield(options,'periodic')
        options.periodic = false;
      end
      if ~isfield(options,'u_const_across_transitions')
        options.u_const_across_transitions = false;
      end
      
      if ~isa(plant,'HybridDrakeSystem')
        error('The plant must be a HybridDrakeSystem');
      end

      obj=obj@NonlinearProgram(0);
      
      obj.N = N;
      obj.M = length(mode_sequence);
      obj.mode_sequence = mode_sequence;
      obj.plant = plant;
      
      % generate a sequence of trajectory optimization problems
      obj.mode_opt = cell(obj.M,1);
      for i=1:obj.M,
        mode = mode_sequence(i);
        obj.mode_opt{i} = traj_opt(plant.getMode(mode), N(i), duration{i}, options);
      end
      
      nvars = 0;
      for i=1:obj.M,
        obj.var_offset(i) = nvars;
                
        % add decision variables
        obj = obj.addDecisionVariable(obj.mode_opt{i}.num_vars);
        
        nvars = nvars + obj.mode_opt{i}.num_vars;
      end
      
      obj = obj.addJumpConstraints();    
      
            
      if options.periodic
        nX = plant.modes{mode_sequence(1)}.getNumStates;
        nU = plant.modes{mode_sequence(1)}.getNumInputs;
        if nX ~= plant.modes{mode_sequence(end)}.getNumStates
          error('For periodic option, the first mode must have the same number of states as the last mode')
        end
        if nU ~= plant.modes{mode_sequence(end)}.getNumInputs
          error('For periodic option, the first mode must have the same number of control inputs as the last mode')
        end
        
        R_periodic = [speye(nX+nU) -speye(nX+nU)];
        periodic_constraint = LinearConstraint(zeros(nX+nU,1),zeros(nX+nU,1),R_periodic);
        periodic_inds = [obj.var_offset(1) + obj.mode_opt{1}.x_inds(:,1);...
          obj.var_offset(1) + obj.mode_opt{1}.u_inds(:,1);...
          obj.var_offset(end) + obj.mode_opt{end}.x_inds(:,end);...
          obj.var_offset(end) + obj.mode_opt{end}.u_inds(:,end)];
        obj = obj.addConstraint(periodic_constraint,periodic_inds);
      end
      
      if options.u_const_across_transitions
        for i=1:obj.M-1,
          nU = plant.modes{mode_sequence(i)}.getNumInputs;
          if nU ~= plant.modes{mode_sequence(i+1)}.getNumInputs
            error('For u_const_across_transitions options, each mode must have the same number of inputs')
          end
          
          u_const_constraint = LinearConstraint(zeros(nU,1),zeros(nU,1),[speye(nU) -speye(nU)]);
          u_const_inds = [obj.var_offset(i) + obj.mode_opt{i}.u_inds(:,end); obj.var_offset(i+1) + obj.mode_opt{i+1}.u_inds(:,1)];
          obj = obj.addConstraint(u_const_constraint,u_const_inds);
        end
      end
    end
    
    function obj = addModeConstraint(obj,mode_ind,varargin)
      if obj.is_compiled
        error('Cannot add mode constraints after the program has been compiled');
      end
      obj.mode_opt{mode_ind} = obj.mode_opt{mode_ind}.addConstraint(varargin{:});
    end
    
    function obj = addModeStateConstraint(obj,mode_ind,varargin)
      if obj.is_compiled
        error('Cannot add mode constraints after the program has been compiled');
      end
      obj.mode_opt{mode_ind} = obj.mode_opt{mode_ind}.addStateConstraint(varargin{:});
    end
    
    function obj = addModeInputConstraint(obj,mode_ind,varargin)
      if obj.is_compiled
        error('Cannot add mode constraints after the program has been compiled');
      end
      obj.mode_opt{mode_ind} = obj.mode_opt{mode_ind}.addInputConstraint(varargin{:});
    end
    
    function obj = addModeInitialCost(obj,mode_ind,initial_cost)
      if obj.is_compiled
        error('Cannot add mode constraints after the program has been compiled');
      end
      obj.mode_opt{mode_ind} = obj.mode_opt{mode_ind}.addInitialCost(initial_cost);
    end
    
    function obj = addModeFinalCost(obj,mode_ind,final_cost_function)
      if obj.is_compiled
        error('Cannot add mode constraints after the program has been compiled');
      end
      obj.mode_opt{mode_ind} = obj.mode_opt{mode_ind}.addFinalCost(final_cost_function);
    end
    
    function obj = addModeRunningCost(obj,mode_ind,running_cost_function)
      if obj.is_compiled
        error('Cannot add mode constraints after the program has been compiled');
      end
      obj.mode_opt{mode_ind} = obj.mode_opt{mode_ind}.addRunningCost(running_cost_function);
    end
    
    function obj = compile(obj)
      if obj.is_compiled
        warning('HybridTrajectoryOptimization was already compiled! Skipping compilation, but getting here may be a result of an error')
        return
      end
      ndata = 0;
      for i=1:obj.M,
        sub_mode = obj.mode_opt{i};
        
        obj.shared_data_offset(i) = ndata;
        
        % add shared data functions
        for j=1:length(sub_mode.shared_data_functions),
          xind = cellfun(@(x) plus(x,obj.var_offset(i)), sub_mode.shared_data_xind_cell{j},'UniformOutput',false);
          obj = obj.addSharedDataFunction(sub_mode.shared_data_functions{j},xind);
        end
        
        % add nonlinear constraints
        for j=1:length(sub_mode.nlcon),
          %offset xind and dataind
          xind = cellfun(@(x) plus(x,obj.var_offset(i)), sub_mode.nlcon_xind{j},'UniformOutput',false);
          dataind = obj.shared_data_offset(i) + sub_mode.nlcon_dataind{j};
          obj = obj.addNonlinearConstraint(sub_mode.nlcon{j},xind,dataind);
        end
        
        % add linear constraints
        nin = length(sub_mode.bin);
        obj.Ain = [obj.Ain;zeros(nin, obj.var_offset(i)) sub_mode.Ain zeros(nin,obj.num_vars - obj.var_offset(i) - size(sub_mode.Ain,2))];
        obj.bin = [obj.bin;sub_mode.bin];
        
        neq = length(sub_mode.beq);
        obj.Aeq = [obj.Aeq;zeros(neq, obj.var_offset(i)) sub_mode.Aeq zeros(neq,obj.num_vars - obj.var_offset(i) - size(sub_mode.Aeq,2))];
        obj.beq = [obj.beq;sub_mode.beq];
        
        % add bounding box constraints
        to_ind = obj.var_offset(i)+1:obj.var_offset(i) + sub_mode.num_vars;
        obj.x_lb(to_ind) = max(obj.x_lb(to_ind), sub_mode.x_lb);
        obj.x_ub(to_ind) = min(obj.x_ub(to_ind), sub_mode.x_ub);
        
        % add cost
        for j=1:length(obj.mode_opt{i}.cost),
          %offset xind and dataind
          xind = cellfun(@(x) plus(x,obj.var_offset(i)), sub_mode.cost_xind_cell{j},'UniformOutput',false);
          dataind = obj.shared_data_offset(i) + sub_mode.cost_dataind{j};
          obj = obj.addCost(sub_mode.cost{j},xind,dataind);
        end
        
        
        ndata = ndata + length(sub_mode.shared_data_functions);
      end
      obj.is_compiled = true;
    end
    
    function obj = addJumpConstraints(obj)
      for i=1:obj.M,
        mode_in = obj.plant.modes{obj.mode_sequence(i)};
        
        if i == obj.M,
          transition_ind = -1;
        elseif any(obj.plant.target_mode{i} == 0)
          if length(obj.plant.target_mode{i}) > 1
            error('Not handling this case right now, better to specify the target modes of all transitions')
          end
          transition_ind = 1;
        else
          transition_ind = find(obj.plant.target_mode{i} == obj.mode_sequence(i+1));
        end
        
        if i < obj.M          
          mode_out = obj.plant.modes{obj.mode_sequence(i+1)};
          
          % add transition constraint, xp = transition(xm,u)
          transition_fun = @(xm,u,xp) transition_constraint_fun(obj.plant,obj.plant.transition{i}{transition_ind}, ...
            obj.mode_sequence(i),xm,u,xp);
          
          jump_con = FunctionHandleConstraint(zeros(mode_out.getNumStates,1), zeros(mode_out.getNumStates,1), ...
            mode_out.getNumStates + mode_in.getNumStates + mode_in.getNumInputs, transition_fun);
          
          jump_xind{1} = obj.mode_opt{i}.x_inds(:,end) + obj.var_offset(i); % xm
          jump_xind{2} = obj.mode_opt{i}.u_inds(:,end) + obj.var_offset(i); % u
          jump_xind{3} = obj.mode_opt{i+1}.x_inds(:,1) + obj.var_offset(i+1); % xp
          
          obj = obj.addConstraint(jump_con, jump_xind);
        end
        
        % add guard constraints, 0 = guard(xm,u) for the transition and
        % 0 >= guard(xm,u) for everything else
        for j=1:length(obj.plant.guard{i}),
          for k=1:obj.N(i),
            if k==obj.N(i) && j==transition_ind,
              guard_ub = 0;
            else
              guard_ub = inf;
            end
            
            guard_fun = @(x,u) guard_constraint_fun(obj.plant,obj.plant.guard{i}{j},x,u);
            guard_con = FunctionHandleConstraint(0,guard_ub,mode_in.getNumStates + mode_in.getNumInputs, guard_fun);
            guard_xind{1} = obj.mode_opt{i}.x_inds(:,k) + obj.var_offset(i); % xm
            guard_xind{2} = obj.mode_opt{i}.u_inds(:,k) + obj.var_offset(i); % u
            obj = obj.addConstraint(guard_con,guard_xind);
          end
        end
      end
      
      function [f,df] = transition_constraint_fun(plant,transition_fun,init_mode,xm,u,xp)
        [xp_fun,~,~,dxp] = transition_fun(plant,init_mode,0,xm,u);
        f = xp - xp_fun;
        df = [-dxp(:,3:end) eye(length(xp))];
      end
      
      function [f,df] = guard_constraint_fun(plant,guard_fun,x,u)
        [f,dg] = guard_fun(plant,0,x,u);
        df = dg(:,2:end);
      end
    end
    
    function z0 = getInitialVars(obj,t_init,traj_init)
      z0 = cell(obj.M,1);
      for i=1:obj.M,
        z0{i} = obj.mode_opt{i}.getInitialVars(t_init{i},traj_init{i});
      end
      z0 = cell2mat(z0);
    end
    
    function [xtraj,utraj,z,F,info] = solveTraj(obj,t_init,traj_init)
      % Solve the nonlinear program and return resulting trajectory
      % @param t_init initial timespan for solution.  can be a vector of
      % length obj.N specifying the times of each segment, or a scalar
      % indicating the final time.
      % @param traj_init (optional) a structure containing Trajectory
      % objects specifying the initial guess for the system inputs
      %    traj_init.u , traj_init.x, ...
      % @default small random numbers
      
      if ~obj.is_compiled
        warning('HybridTrajectoryOptimization is not compiled, doing so now, but previous references not reflect the compilation. Consider running compile() first.');
        obj = obj.compile();
      end

      z0 = obj.getInitialVars(t_init,traj_init);
      
      [z,F,info] = obj.solve(z0);
      
      for i=1:obj.M,
        utraj{i} = obj.mode_opt{i}.reconstructInputTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        xtraj{i} = obj.mode_opt{i}.reconstructStateTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        
        xtraj{i} = MixedTrajectory({PPTrajectory(foh(xtraj{i}.tspan,[obj.mode_sequence(i) obj.mode_sequence(i)])),xtraj{i}},{1,2:1+xtraj{i}.dim});
        if i > 1
          xtraj{i} = xtraj{i}.shiftTime(xtraj{i-1}.tspan(2));
          utraj{i} = utraj{i}.shiftTime(utraj{i-1}.tspan(2));
        end
        
        xtraj{i} = xtraj{i}.setOutputFrame(obj.plant.getStateFrame);
      end
      xtraj = HybridTrajectory(xtraj);
      utraj = HybridTrajectory(utraj);
    end
  end  
end