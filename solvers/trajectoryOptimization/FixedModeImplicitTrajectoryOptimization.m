classdef FixedModeImplicitTrajectoryOptimization < DirectTrajectoryOptimization
  % phi, lambda
  properties
    nC
    nD % number of friction elements per contact
    
    l_inds % orderered [lambda_N;lambda_f1;lambda_f2;...;gamma] for each contact sequentially
    lfi_inds % nD x nC indexes into lambda for each time step
    lambda_mult
    ljl_inds  % joint limit forces
    jl_lb_ind  % joint indices where the lower bound is finite
    jl_ub_ind % joint indices where the lower bound is finite
    nJL % number of joint limits = length([jl_lb_ind;jl_ub_ind])
  end
  
  properties (Constant)
    FORWARD_EULER = 1;
    BACKWARD_EULER = 2;
    MIDPOINT = 3;  % DEFAULT
    MIXED = 4;   % matched to TimeSteppingRigidBodyManipulator. Forward on qd, backward on q
  end
  
  methods
    function obj = FixedModeImplicitTrajectoryOptimization(plant,N_vec,duration,mode_seq_vec,options)
      if nargin<4, options=struct(); end
            
      if ~isfield(options,'lambda_mult')
        options.lambda_mult = 1;
      end
      if ~isfield(options,'lambda_jl_mult')
        options.lambda_jl_mult = 1;
      end
      if ~isfield(options,'active_collision_options')
        options.active_collision_options.terrain_only = true;
      end
      if ~isfield(options,'integration_method')
        options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
      end
      
      if ~isfield(options,'use_joint_limits')
        options.use_joint_limits = false;
      end
      
      % clear time option, add time constraint after constructor
      % take arguments in as arrays, then rebuild
      options.time_option = 2; % all timesteps independent
      N = 1+sum(N_vec);
      valuecheck(length(N_vec), size(mode_seq_vec,2));
      
      mode_seq = repmat(mode_seq_vec(:,1),1,N_vec(1));
      for i=2:length(N_vec),
        mode_seq = [mode_seq, repmat(mode_seq_vec(:,i),1,N_vec(i))];
      end      
      options.mode_seq = mode_seq;
      
      obj = obj@DirectTrajectoryOptimization(plant,N,duration,options);

      % time constraints: all timesteps constant per mode
      for i=1:length(N_vec),
        A_time = [eye(N_vec(i)-1) zeros(N_vec(i)-1,1)] - [zeros(N_vec(i)-1,1) eye(N_vec(i)-1)];
        time_constraint = LinearConstraint(zeros(N_vec(i)-1,1),zeros(N_vec(i)-1,1),A_time);
        obj = obj.addConstraint(time_constraint,obj.h_inds(sum(N_vec(1:i-1)) + (1:N_vec(i))));
      end
    end
    
    function obj = addDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nq = obj.plant.getNumPositions();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);      
      
      n_vars = 2*nX + nU + 1 + obj.nC*(2+obj.nD) + obj.nJL;
      cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.dynamics_constraint_fun);
      
      [~,~,~,~,~,~,~,mu] = obj.plant.contactConstraints(zeros(nq,1),false,obj.options.active_collision_options);
      
      sizecheck(obj.options.mode_seq,[obj.nC obj.N-1]);
      
      % set all contact variables to be positive
      obj = obj.addConstraint(BoundingBoxConstraint(zeros(N*obj.nC*(obj.nD+2),1),inf(N*obj.nC*(obj.nD+2),1)),obj.l_inds(:));
      
      for i=1:obj.N-1,
%         dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)];
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)};
        constraints{i} = cnstr;
        
        obj = obj.addConstraint(constraints{i}, dyn_inds{i});
        
        ind_active = find(obj.options.mode_seq(:,i)==true);
        ind_inactive = find(obj.options.mode_seq(:,i)==false);
        Nactive = length(ind_active);
        Ninactive = length(ind_inactive);
        
        % force phi=gamma=0 for active contacts
        if i~=obj.N-1
        if Nactive > 0
          phigamma_cnstr = FunctionHandleConstraint(zeros(Nactive*(obj.nD+1),1),zeros(Nactive*(obj.nD+1),1),nX,@(q,v) obj.phigamma_fun(q,v,ind_active));        
          obj = obj.addConstraint(phigamma_cnstr,{obj.x_inds(1:nq,i+1),obj.x_inds(nq+1:end,i+1)});
        end
        end
        
        % force lambda_z = 0 for inactive contacts
        lz_inds = obj.l_inds(1:obj.nD+2:end,i);        
        obj = obj.addConstraint(BoundingBoxConstraint(zeros(Ninactive,1),zeros(Ninactive,1)),lz_inds(ind_inactive));
        
        
        % friction limits for all contacts
        
        % awkward way to pull out these indices, for (i) lambda_N and
        % lambda_f
        lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
          
        M = zeros(obj.nC,obj.nC*(1+obj.nD));
        for k=1:obj.nC,
          M(k,1 + (k-1)*(1+obj.nD)) = mu(k);
          M(k,(2:obj.nD+1) + (k-1)*(1+obj.nD)) = -ones(obj.nD,1);
        end
        
        friction_cnstr = LinearConstraint(zeros(obj.nC,1),inf(obj.nC,1),M);
        obj = obj.addConstraint(friction_cnstr,lambda_inds);
               
        if obj.nJL > 0
          error('Not yet implemented, but would be trivial')
          % joint limit linear complementarity constraint
          % lambda_jl /perp [q - lb_jl; -q + ub_jl]
          W_jl = zeros(obj.nJL);
          [r_jl,M_jl] = jointLimitConstraints(obj.plant,zeros(nq,1));
          jlcompl_constraints{i} = LinearComplementarityConstraint(W_jl,r_jl,M_jl,obj.options.lincc_mode,obj.options.jlcompl_slack);
          
          obj = obj.addConstraint(jlcompl_constraints{i},[obj.x_inds(1:nq,i+1);obj.ljl_inds(:,i)]);
        end
      end     
    end
    
    % nonlinear contact constraint, phi=0, D*psi=0
    % x = [q;v]
    function [f,df] = phigamma_fun(obj,q,v,inds)
      Nactive = length(inds);
      nq = obj.plant.getNumPositions;
      nv = obj.plant.getNumVelocities;
      [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q,false,obj.options.active_collision_options);
      f = zeros(Nactive*(1+obj.nD),1);
      df = zeros(Nactive*(1+obj.nD),nq+nv);
      
      f(1:1+obj.nD:end) = phi(inds);
      df(1:1+obj.nD:end,1:nq) = n(inds,:);
      for j=1:obj.nD,
        f(1+j:1+obj.nD:end) = D{j}(inds,:)*v;
        df(1+j:1+obj.nD:end,nq+(1:nv)) = D{j}(inds,:);%d/dv
        
        ddq = matGradMult(dD{j},v);
        df(1+j:1+obj.nD:end,1:nq) = ddq(inds,:);%d/dq
      end
    end
    
     function [f,df] = dynamics_constraint_fun(obj,h,x0,x1,u,lambda,lambda_jl)
        nq = obj.plant.getNumPositions;
        nv = obj.plant.getNumVelocities;
        nu = obj.plant.getNumInputs;
        nl = length(lambda);
        njl = length(lambda_jl);
        
        lambda = lambda*obj.options.lambda_mult;
        lambda_jl = lambda_jl*obj.options.lambda_jl_mult;
        
        assert(nq == nv) % not quite ready for the alternative
        
        q0 = x0(1:nq);
        v0 = x0(nq+1:nq+nv);
        q1 = x1(1:nq);
        v1 = x1(nq+1:nq+nv);
        
        switch obj.options.integration_method
          case ContactImplicitTrajectoryOptimization.MIDPOINT
            [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics((q0+q1)/2,(v0+v1)/2);
            dH0 = dH/2;
            dC0 = dC/2;
            dB0 = dB/2;
            dH1 = dH/2;
            dC1 = dC/2;
            dB1 = dB/2;
          case ContactImplicitTrajectoryOptimization.FORWARD_EULER
            [H,C,B,dH0,dC0,dB0] = obj.plant.manipulatorDynamics(q0,v0);
            dH1 = zeros(nq^2,2*nq);
            dC1 = zeros(nq,2*nq);
            dB1 = zeros(nq*nu,2*nq);
          case ContactImplicitTrajectoryOptimization.BACKWARD_EULER
            [H,C,B,dH1,dC1,dB1] = obj.plant.manipulatorDynamics(q1,v1);
            dH0 = zeros(nq^2,2*nq);
            dC0 = zeros(nq,2*nq);
            dB0 = zeros(nq*nu,2*nq);
          case ContactImplicitTrajectoryOptimization.MIXED
            [H,C,B,dH0,dC0,dB0] = obj.plant.manipulatorDynamics(q0,v0);
            dH1 = zeros(nq^2,2*nq);
            dC1 = zeros(nq,2*nq);
            dB1 = zeros(nq*nu,2*nq);
        end
        
        BuminusC = B*u-C; 
        if nu>0,           
          dBuminusC0 = matGradMult(dB0,u) - dC0;
          dBuminusC1 = matGradMult(dB1,u) - dC1;
        else
          dBuminusC0 = -dC0;
          dBuminusC1 = -dC1;
        end
        
        switch obj.options.integration_method
          case ContactImplicitTrajectoryOptimization.MIDPOINT
            % q1 = q0 + h*v1
            fq = q1 - q0 - h*(v0 + v1)/2;
            dfq = [-(v1+v0)/2, -eye(nq), -h/2*eye(nq), eye(nq), -h/2*eye(nq) zeros(nq,nu+nl+njl)];
          case ContactImplicitTrajectoryOptimization.FORWARD_EULER
            % q1 = q0 + h*v1
            fq = q1 - q0 - h*v0;
            dfq = [-v0, -eye(nq), -h*eye(nq), eye(nq), zeros(nq,nv) zeros(nq,nu+nl+njl)];
          case ContactImplicitTrajectoryOptimization.BACKWARD_EULER
            % q1 = q0 + h*v1
            fq = q1 - q0 - h*v1;
            dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
          case ContactImplicitTrajectoryOptimization.MIXED
            fq = q1 - q0 - h*v1;
            dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
        end
        
        
        % H*v1 = H*v0 + h*(B*u - C) + n^T lambda_N + d^T * lambda_f
        fv = H*(v1 - v0) - h*BuminusC;
        % [h q0 v0 q1 v1 u l ljl]
        
        dfv = [-BuminusC, zeros(nv,nq), -H, zeros(nv,nq), H,-h*B, zeros(nv,nl+njl)] + ...
          [zeros(nv,1) matGradMult(dH0,v1-v0)-h*dBuminusC0 matGradMult(dH1,v1-v0)-h*dBuminusC1 zeros(nv,nu+nl+njl)];
        
        if nl>0
          [phi,~,~,~,~,~,~,~,n,D,dn,dD] = obj.plant.contactConstraints(q1,false,obj.options.active_collision_options);
          % construct J and dJ from n,D,dn, and dD so they relate to the
          % lambda vector
          J = zeros(nl,nq);
          J(1:2+obj.nD:end,:) = n;
          dJ = zeros(nl*nq,nq);
          dJ(1:2+obj.nD:end,:) = dn;
          
          for j=1:length(D),
            J(1+j:2+obj.nD:end,:) = D{j};
            dJ(1+j:2+obj.nD:end,:) = dD{j};
          end

          fv = fv - J'*lambda;
          dfv(:,2+nq+nv:1+2*nq+nv) = dfv(:,2+nq+nv:1+2*nq+nv) - matGradMult(dJ,lambda,true);
          dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl) = -J'*obj.options.lambda_mult;
        end
        
        if njl>0
          [~,J_jl] = jointLimitConstraints(obj.plant,q1);
          
          fv = fv - J_jl'*lambda_jl;
          dfv(:,2+2*nq+2*nv+nu+nl:1+2*nq+2*nv+nu+nl+njl) = -J_jl'*obj.options.lambda_jl_mult;
        end
        
        f = [fq;fv];
        df = [dfq;dfv];
      end
    
    function [xtraj,utraj,ltraj,ljltraj,z,F,info] = solveTraj(obj,t_init,traj_init)
      [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
      t = [0; cumsum(z(obj.h_inds))];
      if obj.nC>0
        ltraj = PPTrajectory(foh(t,reshape(z(obj.l_inds),[],obj.N)));
      else
        ltraj = [];
      end
      if obj.nJL>0
        ljltraj = PPTrajectory(foh(t,reshape(z(obj.ljl_inds),[],obj.N)));
      else
        ljltraj = [];
      end
    end
    
    function obj = setupVariables(obj,N)
      obj = setupVariables@DirectTrajectoryOptimization(obj,N);
      obj.nC = obj.plant.getNumContactPairs;
      [~,normal,d] = obj.plant.contactConstraints(zeros(obj.plant.getNumPositions,1));
      obj.nD = 2*length(d);
      assert(size(normal,2) == obj.nC); % just a double check
      
      nContactForces = obj.nC*(2 + obj.nD);    
      
      obj.l_inds = reshape(obj.num_vars + (1:N * nContactForces),nContactForces,N);
      obj = obj.addDecisionVariable(N * nContactForces);
      
      obj.lfi_inds = zeros(obj.nD,obj.nC);
      for i=1:obj.nC,
        obj.lfi_inds(:,i) = (2:1+obj.nD)' + (i-1)*(2+obj.nD)*ones(obj.nD,1);
      end            
      
      if obj.options.use_joint_limits
        obj.nJL = obj.plant.getNumJointLimitConstraints();
        
        obj.ljl_inds = reshape(obj.num_vars + (1:N * obj.nJL),obj.nJL,N);
        
        % joint limit constraints
        [jl_lb,jl_ub] = obj.plant.getJointLimits();
        obj.jl_lb_ind = find(jl_lb ~= -inf);
        obj.jl_ub_ind = find(jl_ub ~= inf);
        
        obj = obj.addDecisionVariable(N * obj.nJL);
      else
        obj.nJL = 0;
        obj.ljl_inds = reshape(obj.num_vars + (1:N * obj.nJL),obj.nJL,N);
      end
    end
    
    % evaluates the initial trajectories at the sampled times and
    % constructs the nominal z0. Overwrite to implement in a different
    % manner
    function z0 = getInitialVars(obj,t_init,traj_init)
      if isscalar(t_init)
        t_init = linspace(0,t_init,obj.N);
      elseif length(t_init) ~= obj.N
        error('The initial sample times must have the same length as property N')
      end
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(t_init);

      if isfield(traj_init,'u')
        z0(obj.u_inds) = traj_init.u.eval(t_init);
      else
        nU = getNumInputs(obj.plant);
        z0(obj.u_inds) = 0.01*randn(nU,obj.N);
      end
      
      if isfield(traj_init,'x')
        z0(obj.x_inds) = traj_init.x.eval(t_init);
      else
        if ~isfield(traj_init,'u')
          traj_init.u = setOutputFrame(PPTrajectory(foh(t_init,reshape(z0(obj.u_inds),nU,obj.N))),getInputFrame(obj.plant));
        end
        
        % todo: if x0 and xf are equality constrained, then initialize with
        % a straight line from x0 to xf (this was the previous behavior)
        
        %simulate
        sys_ol = cascade(traj_init.u,obj.plant);
        [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)]);
        z0(obj.x_inds) = x_sim.eval(t_init);
      end
      
      if obj.nC > 0
        if isfield(traj_init,'l')
          z0(obj.l_inds) = traj_init.l.eval(t_init);
        else
          z0(obj.l_inds) = 0;
        end
      end
      if obj.nJL > 0
        if isfield(traj_init,'ljl')
          z0(obj.ljl_inds) = traj_init.ljl.eval(t_init);
        else          
          z0(obj.ljl_inds) = 0;
        end
      end          
    end
      
    function obj = addRunningCost(obj,running_cost_function)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      running_cost = FunctionHandleObjective(1+nX+nU,running_cost_function);
      for i=1:obj.N-1,
        obj = obj.addCost(running_cost,{obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i)});
      end
    end
    
    function utraj = reconstructInputTrajectory(obj,z)
      % use a zero order hold
      t = [0; cumsum(z(obj.h_inds))];

      if size(obj.u_inds,1)>0
        u = reshape(z(obj.u_inds),[],obj.N);
        utraj = PPTrajectory(zoh(t,u));
        utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
      else
        utraj=[];
      end
    end
    
  end
end
