classdef ConstrainedHybridTrajectoryOptimization < NonlinearProgram
  % ConstrainedHybridTrajectoryOptimization
  % 
  % Contact specific implementation of the hybrid DIRCON algorithm from
  % "Optimization and stabilization of trajectories for constrained
  %  dynamical systems" by Posa, Kuindersma and Tedrake 2016.
  %
  %  Utilizes the ContactConstrainedDircolTrajectoryOptimization class to
  %  construct the optimizatino programs for each mode
  % 
  %  This class was constructed similarly to the
  %  HybridTrajectoryOptimization class, but it does NOT derive from that
  %  class.
  %
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
  %  traj_opt = ConstrainedHybridTrajectoryOptimization(...);
  %  traj_opt = traj_opt.addConstraint(...)
  %  traj_opt = traj_opt.addModeConstraint(...)
  %  ...
  %  traj_opt = traj_opt.compile()
  %  traj_opt.solveTraj()
  %
  % Constructor: 
  %  obj = ConstrainedHybridTrajectoryOptimization(plant,mode_indices,N,duration,options)
  %   @param mode_indices : A cell array of vectors corresponding to
  %   contact indices for ContactConstrainedDircolTrajectoryOptimization
  %   options.periodic : Enforces a periodicity constraint between last and
  %     first states, defaults to false
  %   options.u_const_across_transitions : If true, control input must be
  %     constant across hybrid transitions. Defaults to false
  %   options.mode_options : A struct of options to pass along to each
  %     suboptimization program.
  
  properties %(SetAccess=protected)
    N
    M
    nD        %size of linearized friction cone
    contact_sequence
    mode_opt
    mixed_constraint_args
    var_offset
    shared_data_offset
    plant
    is_compiled = false;
    Lambda_inds
  end
  
  methods
    function obj = ConstrainedHybridTrajectoryOptimization(plant,mode_indices,N,duration,options)
      if nargin < 5
        options = struct();
      end
      if ~isfield(options,'periodic')
        options.periodic = false;
      end
      if ~isfield(options,'u_const_across_transitions')
        options.u_const_across_transitions = false;
      end      

      obj=obj@NonlinearProgram(0);
      
      obj.N = N;
      obj.M = length(mode_indices);
      obj.contact_sequence = mode_indices;
      obj.plant = plant;
      
      % generate a sequence of trajectory optimization problems
      obj.mode_opt = cell(obj.M,1);
      for i=1:obj.M,
        mode_options = options;
        if isfield(options,'mode_options')
          fields = fieldnames(options.mode_options{i});
          for j=1:numel(fields),
            mode_options.(fields{j}) = options.mode_options{i}.(fields{j});
          end
        end
        if i ~= 1,
%           mode_options.constrain_phi_start = false;
        end
        if i ~= obj.M,
%           mode_options.constrain_phi_end = false;
        end
        obj.mode_opt{i} = ContactConstrainedDircolTrajectoryOptimization(plant,N(i),duration{i},mode_indices{i},mode_options);
      end
      
      nvars = 0;
      for i=1:obj.M,
        obj.var_offset(i) = nvars;
                
        % add decision variables
        obj = obj.addDecisionVariable(obj.mode_opt{i}.num_vars);
        
        nvars = nvars + obj.mode_opt{i}.num_vars;
      end
      
      % Add Lambda variables
      [~,normal,d] = plant.contactConstraints(zeros(obj.plant.getNumPositions,1));
      obj.nD = 2*length(d);
      
      for i=1:obj.M-1,
        if any(setdiff(mode_indices{i+1},mode_indices{i}))
          nLambda = length(mode_indices{i+1})*(obj.nD+1);
          obj.Lambda_inds{i} = nvars + (1:nLambda)';
          obj = obj.addDecisionVariable(nLambda);
          nvars = nvars + nLambda;
          
          % todo: get the real mu here
          mu = 1;
          obj = obj.addConstraint(BoundingBoxConstraint(zeros(nLambda,1),inf(nLambda,1)),obj.Lambda_inds{i});
          
          A_fric = zeros(length(mode_indices{i+1})*obj.nD,nLambda);
          for j=1:length(mode_indices{i+1}),
            A_fric((j-1)*obj.nD + (1:obj.nD),(j-1)*(obj.nD+1) + 1) = mu*ones(obj.nD,1);
            A_fric((j-1)*obj.nD + (1:obj.nD),(j-1)*(obj.nD+1) + (2:obj.nD+1)) = -eye(obj.nD);
          end
          
          obj = obj.addConstraint(LinearConstraint(zeros(size(A_fric,1),1),inf(size(A_fric,1),1),A_fric),obj.Lambda_inds{i});
        else
          obj.Lambda_inds{i} = [];
        end
      end
      
      obj = obj.addJumpConstraints();    
                  
      if options.periodic
        nX = plant.getNumStates;
        nU = plant.getNumInputs;
                
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
          nU = plant.getNumInputs;
          
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
      nq = obj.plant.getNumPositions;
      nv = obj.plant.getNumVelocities;
      for i=2:obj.M,
        % linear constraint for positions
        lin_con = LinearConstraint(zeros(nq,1),zeros(nq,1),[eye(nq) -eye(nq)]);
        obj = obj.addConstraint(lin_con,[obj.mode_opt{i-1}.x_inds(1:nq,end) + obj.var_offset(i-1);...
          obj.mode_opt{i}.x_inds(1:nq,1) + obj.var_offset(i)]);
        
        % nonlinear constraint for velocities
        all_contact_inds = unique([obj.contact_sequence{i-1};obj.contact_sequence{i}]);
        all_contact_inds = [];
        jump_fun = @(q,vm,vp,Lambda) obj.impulse_jump_fun(obj.contact_sequence{i},all_contact_inds,q,vm,vp,Lambda);
        jump_con = FunctionHandleConstraint(zeros(nv+length(all_contact_inds),1),zeros(nv+length(all_contact_inds),1),nq+nv*2+length(obj.Lambda_inds{i-1}),jump_fun);
        jump_vars = {obj.mode_opt{i-1}.x_inds(1:nq,end) + obj.var_offset(i-1),...
          obj.mode_opt{i-1}.x_inds(nq+1:end,end) + obj.var_offset(i-1),...
          obj.mode_opt{i}.x_inds(nq+1:end,1) + obj.var_offset(i),obj.Lambda_inds{i-1}};
        
        obj = obj.addConstraint(jump_con,jump_vars);
      end
    end       
    
    function [f,df] = impulse_jump_fun(obj,new_contact_inds,all_contact_inds,q,vm,vp,Lambda)
      %also includes position constraints using all_contact_inds
      nq = obj.plant.getNumPositions;
      nv = obj.plant.getNumVelocities;
      nl = length(Lambda);
      
      [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q,false,struct('terrain_only',true));
      if nl > 0
        [H,C,B,dH] = obj.plant.manipulatorDynamics(q,vm);
        
        grad_inds = kron(ones(nq,1),new_contact_inds) + kron((0:nq-1)',length(phi)*ones(length(new_contact_inds),1));
        
        J = zeros(nl,nq);
        J(1:1+obj.nD:end,:) = n(new_contact_inds,:);
        dJ = zeros(nl*nq,nq);
        dJ(1:1+obj.nD:end,:) = dn(grad_inds,:);
        
        for j=1:length(D),
          J(1+j:1+obj.nD:end,:) = D{j}(new_contact_inds,:);
          dJ(1+j:1+obj.nD:end,:) = dD{j}(grad_inds,:);
        end
        
        f = [phi(all_contact_inds); H*(vp - vm) - J'*Lambda];
        df = [n(all_contact_inds,:), zeros(length(all_contact_inds), 2*nv + nl);
          [matGradMult(dH,vp-vm)-[matGradMult(dJ,Lambda,true) H], H, -J']];
      else
        f = [phi(all_contact_inds); vp - vm];
        df = [n(all_contact_inds,:), zeros(length(all_contact_inds), 2*nv + nl);...
          zeros(nq) -eye(nq) eye(nq)];
      end      
    end
    
    function z0 = getInitialVars(obj,t_init,traj_init)
      z0 = cell(obj.M,1);
      for i=1:obj.M,
        z0{i} = obj.mode_opt{i}.getInitialVars(t_init{i},traj_init.mode{i});
      end
      if isfield(traj_init,'L')
        for i=1:obj.M-1,
          sizecheck(traj_init.L{i},obj.Lambda_inds{i});
          z0{end+1} = traj_init.L{i};
        end
      else
        for i=1:obj.M-1,
          z0{end+1} = zeros(size(obj.Lambda_inds{i}));
        end
      end
      z0 = cell2mat(z0);
    end
    
    
    function [xtraj,utraj,ltraj,z,F,info] = solveTrajFromZ(obj,z0)
      if ~obj.is_compiled
        warning('HybridTrajectoryOptimization is not compiled, doing so now, but previous references not reflect the compilation. Consider running compile() first.');
        obj = obj.compile();
      end

%       obj = obj.setScalesFromX(z0);
      [z,F,info] = obj.solve(z0);
      
      for i=1:obj.M,
        utraj{i} = obj.mode_opt{i}.reconstructInputTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        xtraj{i} = obj.mode_opt{i}.reconstructStateTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        ltraj{i} = obj.mode_opt{i}.reconstructForceTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        
%         xtraj{i} = MixedTrajectory({PPTrajectory(foh(xtraj{i}.tspan,[obj.mode_sequence(i) obj.mode_sequence(i)])),xtraj{i}},{1,2:1+xtraj{i}.dim});
        if i > 1
          xtraj{i} = xtraj{i}.shiftTime(xtraj{i-1}.tspan(2));
          utraj{i} = utraj{i}.shiftTime(utraj{i-1}.tspan(2));
          if ~isempty(ltraj{i})
            ltraj{i} = ltraj{i}.shiftTime(xtraj{i-1}.tspan(2));
          end
        end
        
        xtraj{i} = xtraj{i}.setOutputFrame(obj.plant.getStateFrame);
      end
    end
    
    
    function [xtraj,utraj,ltraj,z,F,info] = solveTraj(obj,t_init,traj_init)
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
%       obj = obj.setScalesFromX(z0);
      [z,F,info] = obj.solve(z0);
      
      for i=1:obj.M,
        utraj{i} = obj.mode_opt{i}.reconstructInputTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        xtraj{i} = obj.mode_opt{i}.reconstructStateTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        ltraj{i} = obj.mode_opt{i}.reconstructForceTrajectory(z(obj.var_offset(i)+1:obj.var_offset(i)+obj.mode_opt{i}.num_vars));
        
%         xtraj{i} = MixedTrajectory({PPTrajectory(foh(xtraj{i}.tspan,[obj.mode_sequence(i) obj.mode_sequence(i)])),xtraj{i}},{1,2:1+xtraj{i}.dim});
        if i > 1
          xtraj{i} = xtraj{i}.shiftTime(xtraj{i-1}.tspan(2));
          utraj{i} = utraj{i}.shiftTime(utraj{i-1}.tspan(2));
          if ~isempty(ltraj{i})
            ltraj{i} = ltraj{i}.shiftTime(xtraj{i-1}.tspan(2));
          end
        end
        
        xtraj{i} = xtraj{i}.setOutputFrame(obj.plant.getStateFrame);
      end
%       xtraj = HybridTrajectory(xtraj);
%       utraj = HybridTrajectory(utraj);
    end
  end  
end