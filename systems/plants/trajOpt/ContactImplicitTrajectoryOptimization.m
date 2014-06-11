classdef ContactImplicitTrajectoryOptimization < TrajectoryOptimization
  % phi, lambda
  properties
    nC
    nD % number of friction elements per contact
    l_inds % orderered [lambda_N;lambda_f1;lambda_f2;...;gamma] for each contact sequentially
    lfi_inds % nD x nC indexes into lambda for each time step
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
      lincompl_constraints = cell(N-1,1);
      nonlincompl_constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + nU + 1 + obj.nC*(2+obj.nD);
      dynfun = @(z) dynamics_constraint_fun(z(1),z(2:nX+1),z(nX+2:2*nX+1),z(2*nX+2:2*nX+nU+1),z(2*nX+nU+2:end));
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,dynfun);
      
      x0 = zeros(nX,1);
      x1 = zeros(nX,1);
      x1(2) = 1; x0(2) = 1;
      l = zeros(obj.nC*(2+obj.nD),1);
      l(1:(2+obj.nD):end) = 1;
      u = zeros(3,1);
      h = rand;

      [~,~,~,~,~,~,~,mu] = obj.plant.contactConstraints(zeros(obj.plant.getNumPositions,1));
      
      for i=1:obj.N-1,        
        dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i)];
        constraints{i} = cnstr;
        
        obj = obj.addNonlinearConstraint(constraints{i}, dyn_inds{i});
        
        % indices for (i) gamma
        gamma_inds = obj.l_inds(obj.nD+2:obj.nD+2:end,i);
        % awkward way to pull out these indices, for (i) lambda_N and
        % lambda_f
        lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
        
        % indices for (i) lambda_N
        lambdaN_inds = obj.l_inds(1:2+obj.nD:end,i);
        % indices for (i) lambdaf and gamma
        
        lambdaf_gamma_inds = obj.l_inds(repmat((2:2+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
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
        
        lincompl_constraints{i} = LinearComplementarityConstraint(W,r,M,obj.options.lincc_mode,0*obj.options.compl_slack);
        

        
        obj = obj.addManagedConstraints(lincompl_constraints{i},[lambda_inds;gamma_inds]);
      end
      
      function [f,df] = dynamics_constraint_fun(h,x0,x1,u,lambda)
        nq = obj.plant.getNumPositions;
        nv = obj.plant.getNumVelocities;
        nu = obj.plant.getNumInputs;
        nl = length(lambda);
        lambda_mult = obj.plant.getMass;
        lambda = lambda*lambda_mult;
        
        assert(nq == nv) % not quite ready for the alternative
        
        q0 = x0(1:nq);
        v0 = x0(nq+1:nq+nv);
        q1 = x1(1:nq);
        v1 = x1(nq+1:nq+nv);
        
        [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics(q1,v1,false);
        
        
        [phi,~,~,~,~,~,~,~,n,D,dn,dD] = obj.plant.contactConstraints(q1,false,struct('terrain_only',true));
        % construct J and dJ from n,D,dn, and dD so they relate to the
        % lambda vector
        J = zeros(nl,nq);
        J(1:2+obj.nD:end,:) = n;
        dJ = zeros(nl*nq,nq);
        dJ(1:2+obj.nD:end,:) = dn;
        
        % q1 = q0 + h*v1
        fq = q1 - q0 - h*v1;
        dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl)];
        
        % H*v1 = H*v0 + h*(B*u - C) + n^T lambda_N + d^T * lambda_f
        fv = H*(v1 - v0) - h*(B*u - C) - J'*lambda;
        
        % [h q0 v0 q1 v1 u l]
        dfv = [-B*u+C, zeros(nv,nq), -H, -matGradMult(dJ,lambda,true), H,-h*B,-J'] + ...
          [zeros(nv,1+nq+nv) matGradMult(dH,v1-v0)-h*(matGradMult(dB,u) - dC) zeros(nv,nu+nl)];
        
        dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl) = dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl)*lambda_mult;
        
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
    
    function [xtraj,utraj,ltraj,z,F,info] = solveTraj(obj,t_init,traj_init)
      [xtraj,utraj,z,F,info] = solveTraj@TrajectoryOptimization(obj,t_init,traj_init);
      t = [0; cumsum(z(obj.h_inds))];
      ltraj = PPTrajectory(foh(t,reshape(z(obj.l_inds),[],obj.N)));
    end
    
    function obj = setupVariables(obj,N)
      obj = setupVariables@TrajectoryOptimization(obj,N);
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
        z0(obj.l_inds(:,i)) = traj_init.l.eval(t_init(i));
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