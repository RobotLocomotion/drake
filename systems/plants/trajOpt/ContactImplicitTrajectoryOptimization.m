classdef ContactImplicitTrajectoryOptimization < TrajectoryOptimization
  % phi, lambda
  properties
    nC
    nD % number of friction elements per contact
    l_inds % orderered [lambda_N;lambda_f1;lambda_f2;...;gamma] for each contact sequentially
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
      
      x0 = randn(18,1);
      x1 = randn(18,1);
      l = 0*randn(12,1);
      u = randn(3,1);
      h = rand;
      
      [f1,df1] = dynamics_constraint_fun(h,x0,x1,u,l);
      [f2,df2] = geval(@dynamics_constraint_fun,h,x0,x1,u,l,struct('grad_method','numerical'));
      
      for i=1:obj.N-1,        
        dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i)];
        constraints{i} = cnstr;
        
        obj = obj.addNonlinearConstraint(constraints{i}, dyn_inds{i});
        
        %TODO: generate complementarity constraints
      end
      
      function [f,df] = dynamics_constraint_fun(h,x0,x1,u,lambda)
        nq = obj.plant.getNumPositions;
        nv = obj.plant.getNumVelocities;
        nu = obj.plant.getNumInputs;
        nl = length(lambda);
        
        assert(nq == nv) % not quite ready for the alternative
        
        q0 = x0(1:nq);
        v0 = x0(nq+1:nq+nv);
        q1 = x1(1:nq);
        v1 = x1(nq+1:nq+nv);
        
        [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics(q1,v1,false);
        
        [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q1);
        % construct J and dJ from n,D,dn, and dD so they relate to the
        % lambda vector
        J = zeros(nl,nq);
        J(1:2+obj.nD:end,:) = n;
        dJ = zeros(nl*nq,nq);
        dJ(1:2+obj.nD:end,:) = dn;
        for i=1:length(D),
          J(1+i:2+obj.nD:end,:) = D{i};
          dJ(1+i:2+obj.nD:end,:) = dD{i};
        end
        
        % q1 = q0 + h*v1
        fq = q1 - q0 - h*v1;
        dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl)];
        
        % H*v1 = H*v0 + h*(B*u - C) + n^T lambda_N + d^T * lambda_f
        fv = H*(v1 - v0) - h*(B*u - C) - J'*lambda;
        
        % [h q0 v0 q1 v1 u l]
        dfv = [-B*u+C, zeros(nv,nq), -H, -matGradMult(dJ,lambda,true), H,-h*B,-J'] + ...
          [zeros(nv,1+nq+nv) matGradMult(dH,v1-v0)-h*(matGradMult(dB,u) - dC) zeros(nv,nu+nl)];
        
        f = [fq;fv];
        df = [dfq;dfv];
        
        f = H(:);
                df = [zeros(length(f),1+nq+nv), dH, zeros(length(f),nu+nl)];
        
%         f = J'*lambda;
%         df = [zeros(nv,1+nq+nv), matGradMult(dJ,lambda,true), zeros(nv,nv+nu) J'];
%         f = J(:);
%         df = [zeros(length(f),1+nq+nv), dJ, zeros(length(f),nv+nu+nl)];
      end
      
      function [f,df] = nonlincompl_fun(x,z)
        
      end
    end
    
    function obj = setupVariables(obj,N)
      obj = setupVariables@TrajectoryOptimization(obj,N);
      obj.nC = obj.plant.getNumContactPairs;
      [~,normal,d] = obj.plant.contactConstraints(zeros(obj.plant.getNumPositions,1));
      obj.nD = 2*length(d);
      assert(size(normal,2) == obj.nC); % just a double check
      
      nContactForces = obj.nC*(2 + obj.nD);
      
      obj.l_inds = obj.num_vars + (1:N * nContactForces);
      obj.addDecisionVariable(N * nContactForces);
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