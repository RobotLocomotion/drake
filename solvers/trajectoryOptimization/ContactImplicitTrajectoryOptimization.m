classdef ContactImplicitTrajectoryOptimization < DirectTrajectoryOptimization
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
  
  methods
    function obj = ContactImplicitTrajectoryOptimization(plant,N,T_span,varargin)
      obj = obj@DirectTrajectoryOptimization(plant,N,T_span,varargin{:});
      
      if ~isfield(obj.options,'nlcc_mode')
        obj.options.nlcc_mode = 1;
      end
      if ~isfield(obj.options,'lincc_mode')
        obj.options.lincc_mode = 1;
      end
      if ~isfield(obj.options,'compl_slack')
        obj.options.compl_slack = 0;
      end
      if ~isfield(obj.options,'lincompl_slack')
        obj.options.lincompl_slack = 0;
      end
      if ~isfield(obj.options,'jlcompl_slack')
        obj.options.jlcompl_slack = 0;
      end
      if ~isfield(obj.options,'lambda_mult')
        obj.lambda_mult = 1;
      end
      if ~isfield(obj.options,'lambda_jl_mult')
        obj.lambda_mult = 1;
      end
    end
    
    function obj = addDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nq = obj.plant.getNumPositions();
      N = obj.N;
      
      constraints = cell(N-1,1);
      lincompl_constraints = cell(N-1,1);
      nonlincompl_constraints = cell(N-1,1);
      jlcompl_constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);      
      
      n_vars = 2*nX + nU + 1 + obj.nC*(2+obj.nD) + obj.nJL;
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,@dynamics_constraint_fun);
      
      [~,~,~,~,~,~,~,mu] = obj.plant.contactConstraints(zeros(nq,1));
      
      for i=1:obj.N-1,        
%         dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)];
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)};
        constraints{i} = cnstr;
        
        obj = obj.addNonlinearConstraint(constraints{i}, dyn_inds{i});
        
        if obj.nC > 0
          % indices for (i) gamma
          gamma_inds = obj.l_inds(obj.nD+2:obj.nD+2:end,i);
          % awkward way to pull out these indices, for (i) lambda_N and
          % lambda_f
          lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
          
          nonlincompl_constraints{i} = NonlinearComplementarityConstraint(@nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD),obj.options.nlcc_mode,obj.options.compl_slack);
          
          obj = obj.addManagedConstraints(nonlincompl_constraints{i},[obj.x_inds(:,i+1);gamma_inds;lambda_inds]);
          
          % linear complementarity constraint
          %   gamma /perp mu*lambda_N - sum(lambda_fi)
          %
          %  Generate terms W,r,M,gamma_inds so that
          %  gamma = y(gamma_inds)
          %  Wz+Mx+r = mu*lambda_N - sum(lambda_fi)
          r = zeros(obj.nC,1);
          W = zeros(obj.nC,obj.nC);
          M = zeros(obj.nC,obj.nC*(1+obj.nD));
          for k=1:obj.nC,
            M(k,1 + (k-1)*(1+obj.nD)) = mu(k);
            M(k,(2:obj.nD+1) + (k-1)*(1+obj.nD)) = -ones(obj.nD,1);
          end
          
          lincompl_constraints{i} = LinearComplementarityConstraint(W,r,M,obj.options.lincc_mode,obj.options.lincompl_slack);
          obj = obj.addManagedConstraints(lincompl_constraints{i},[lambda_inds;gamma_inds]);
        end
        
        if obj.nJL > 0
          % joint limit linear complementarity constraint
          % lambda_jl /perp [q - lb_jl; -q + ub_jl]
          W_jl = zeros(obj.nJL);
          [r_jl,M_jl] = jointLimitConstraints(obj.plant,zeros(nq,1));
          jlcompl_constraints{i} = LinearComplementarityConstraint(W_jl,r_jl,M_jl,obj.options.lincc_mode,obj.options.jlcompl_slack);
          
          obj = obj.addManagedConstraints(jlcompl_constraints{i},[obj.x_inds(1:nq,i+1);obj.ljl_inds(:,i)]);
        end
      end
      
      function [f,df] = dynamics_constraint_fun(h,x0,x1,u,lambda,lambda_jl)
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
        
        [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics(q1,v1);
        
        
        [phi,~,~,~,~,~,~,~,n,D,dn,dD] = obj.plant.contactConstraints(q1,false,struct('terrain_only',true));
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
        
        [~,J_jl] = jointLimitConstraints(obj.plant,q1);
        
        % q1 = q0 + h*v1
        fq = q1 - q0 - h*v1;
        dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
        
        % H*v1 = H*v0 + h*(B*u - C) + n^T lambda_N + d^T * lambda_f
        fv = H*(v1 - v0) - h*(B*u - C) - J'*lambda - J_jl'*lambda_jl;
        
        % [h q0 v0 q1 v1 u l ljl]
        dfv = [-B*u+C, zeros(nv,nq), -H, -matGradMult(dJ,lambda,true), H,-h*B,-J',-J_jl'] + ...
          [zeros(nv,1+nq+nv) matGradMult(dH,v1-v0)-h*(matGradMult(dB,u) - dC) zeros(nv,nu+nl+njl)];
        
        dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl) = dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl)*obj.options.lambda_mult;
        dfv(:,2+2*nq+2*nv+nu+nl:1+2*nq+2*nv+nu+nl+njl) = dfv(:,2+2*nq+2*nv+nu+nl:1+2*nq+2*nv+nu+nl+njl)*obj.options.lambda_jl_mult;
        
        f = [fq;fv];
        df = [dfq;dfv];
      end
      
      % nonlinear complementarity constraints:
      %   lambda_N /perp phi(q)
      %   lambda_fi /perp gamma + Di*psi(q,v)
      % y = [q;v;gamma]
      % z = [lambda_N;lambda_F1;lambda_f2] (each contact sequentially)
      function [f,df] = nonlincompl_fun(y)
        nq = obj.plant.getNumPositions;
        nv = obj.plant.getNumVelocities;
        x = y(1:nq+nv+obj.nC);
        z = y(nq+nv+obj.nC+1:end);
        gamma = x(nq+nv+1:end);
        
        q = x(1:nq);
        v = x(nq+1:nq+nv);
        
        [phi,~,~,~,~,~,~,~,n,D,~,dD] = obj.plant.contactConstraints(q,false,struct('terrain_only',true));
        
        f = zeros(obj.nC*(1+obj.nD),1);
        df = zeros(obj.nC*(1+obj.nD),nq+nv+obj.nC*(2+obj.nD));
        
        f(1:1+obj.nD:end) = phi;
        df(1:1+obj.nD:end,1:nq) = n;
        for j=1:obj.nD,
          f(1+j:1+obj.nD:end) = gamma+D{j}*v;
          df(1+j:1+obj.nD:end,nq+nv+(1:obj.nC)) = eye(size(D{j},1));  %d/dgamma
          df(1+j:1+obj.nD:end,nq+(1:nv)) = D{j};%d/dv
          df(1+j:1+obj.nD:end,1:nq) = matGradMult(dD{j},v);%d/dq
        end
        
      end
    end
    
    function [xtraj,utraj,ltraj,ljltraj,z,F,info] = solveTraj(obj,t_init,traj_init)
      [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
      t = [0; cumsum(z(obj.h_inds))];
      ltraj = PPTrajectory(foh(t,reshape(z(obj.l_inds),[],obj.N)));
      ljltraj = PPTrajectory(foh(t,reshape(z(obj.ljl_inds),[],obj.N)));
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
      
      obj.nJL = obj.plant.getNumJointLimitConstraints();
      obj.ljl_inds = reshape(obj.num_vars + (1:N * obj.nJL),obj.nJL,N);
      
      % joint limit constraints
      [jl_lb,jl_ub] = obj.plant.getJointLimits();
      obj.jl_lb_ind = find(jl_lb ~= -inf);
      obj.jl_ub_ind = find(jl_ub ~= inf);
      
      obj = obj.addDecisionVariable(N * obj.nJL);
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
        if obj.nC > 0
          z0(obj.l_inds(:,i)) = traj_init.l.eval(t_init(i));
        end
        if obj.nJL > 0
          z0(obj.ljl_inds(:,i)) = traj_init.ljl.eval(t_init(i));
        end
      end
    end
    
    function obj = addRunningCost(obj,running_cost)
      for i=1:obj.N-1,
        obj = obj.addCost(running_cost,{obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i)});
      end
    end
    
  end
end