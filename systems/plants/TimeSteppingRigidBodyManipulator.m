classdef TimeSteppingRigidBodyManipulator < DrakeSystem
  % A discrete time system which simulates (an Euler approximation of) the
  % manipulator equations, with contact / limits resolved using the linear
  % complementarity problem formulation of contact in Stewart96.
  
  properties
    manip  % the CT manipulator
    timestep
    twoD=false
  end
  
  methods
    function obj=TimeSteppingRigidBodyManipulator(manipulator,timestep)
      checkDependency('pathlcp_enabled');
      
      switch class(manipulator)
        case {'char','RigidBodyModel'}
          % then make the corresponding manipulator
          S = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
          warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
          manipulator = RigidBodyManipulator(manipulator);
          warning(S);
        case 'PlanarRigidBodyModel'
          S = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits');
          warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedContactPoints');
          manipulator = PlanarRigidBodyManipulator(manipulator);
          warning(S);
      end
      typecheck(manipulator,{'RigidBodyManipulator','PlanarRigidBodyManipulator'});
      obj = obj@DrakeSystem(0,manipulator.getNumStates(),manipulator.getNumInputs(),manipulator.getNumOutputs(),manipulator.isDirectFeedthrough(),manipulator.isTI());
      obj.manip = manipulator;
      if isa(manipulator,'PlanarRigidBodyManipulator')
        obj.twoD = true;
      end
      
      typecheck(timestep,'double');
      sizecheck(timestep,1);
      obj.timestep = timestep;
      
      obj = setSampleTime(obj,[timestep;0]);
            
      obj = setInputFrame(obj,getInputFrame(obj.manip));
      obj = setStateFrame(obj,getStateFrame(obj.manip));
      obj = setOutputFrame(obj,getOutputFrame(obj.manip));
    end
    
    function x0 = getInitialState(obj)
      x0 = obj.manip.getInitialState();
    end
    
    function [xdn,df] = update(obj,t,x,u)
      % do LCP time-stepping
      num_q = obj.manip.num_q;
      q=x(1:num_q); qd=x((num_q+1):end);
      if (obj.twoD) d=2; else d=3; end 
      h = obj.timestep;

      if (nargout<2)
        [H,C,B] = manipulatorDynamics(obj.manip,q,qd);
        if (obj.num_u>0) tau = B*u - C; else tau = -C; end
      else
        [H,C,B,dH,dC,dB] = manipulatorDynamics(obj.manip,q,qd);
        if (obj.num_u>0) 
          tau = B*u - C;  
          dtau = [zeros(num_q,1), matGradMult(dB,u) - dC, B];
        else
          tau = -C; 
          dtau = [zeros(num_q,1), -dC];;
        end
      end
      
      nL = sum([obj.manip.joint_limit_min~=-inf;obj.manip.joint_limit_max~=inf]); % number of joint limits
      nC = obj.manip.num_contacts;
      nP = d*obj.manip.num_position_constraints;  % number of position constraints
      nV = d*obj.manip.num_velocity_constraints;  
      
      if (nC+nL+nP+nV==0)
        qd_out = qd + h*(H\tau);
        q_out = q + h*qd_out;
        xdn = [q_out; qd_out];
        if (nargout>1) error('need to implement this case'); end
        return;
      end      
      
      % Set up the LCP:
      % z >= 0, Mz + w >= 0, z'*(Mz + w) = 0
      % for documentation below, use slack vars: s = Mz + w >= 0
      %
      % use qn = q + h*qdn
      % where H(q)*(qdn - qd)/h = B*u - C(q) + J(q)'*z
      %  or qdn = qd + H\(h*tau + J'*z)
      %  with z = [h*cL; h*cP; h*cN; h*beta{1}; ...; h*beta{mC}; lambda]
      %
      % and implement equation (7) from Anitescu97, by collecting
      %   J = [JL; JP; n; D{1}; ...; D{mC}; zeros(nC,num_q)]

      if (nC > 0)
        if (nargout>1)
          [phiC,n,D,mu,dn,dD] = obj.manip.contactConstraints(q);  % this is what I want eventually.
          mC = length(D);
          dJ = zeros(nL+nP+(mC+2)*nC,num_q^2);  % was sparse, but reshape trick for the transpose below didn't work
          dJ(nL+nP+(1:nC),:) = reshape(dn,nC,[]);
          dD = cellfun(@(A)reshape(A,size(D{1},1),size(D{1},2)*size(dD{1},2)),dD,'UniformOutput',false);
          dD = vertcat(dD{:});
          dJ(nL+nP+nC+(1:mC*nC),:) = dD;
        else
          [phiC,n,D,mu] = obj.manip.contactConstraints(q);
          mC = length(D);
        end
        J = zeros(nL + nP + (mC+2)*nC,num_q)*q(1); % *q(1) is for taylorvar
        D = vertcat(D{:});
        J(nL+nP+(1:nC),:) = n;
        J(nL+nP+nC+(1:mC*nC),:) = D;
      else
        mC=0;
        J = zeros(nL+nP,num_q);
        if (nargout>1)
          dJ = sparse(nL+nP,num_q^2);
        end
      end
      
      if (nL > 0)
        if (nargout<2)
          [phiL,JL] = obj.manip.jointLimits(q);
        else
          [phiL,JL,dJL] = obj.manip.jointLimits(q);
          dJ(1:nL,:) = dJL;
        end
        J(1:nL,:) = JL;
      end
      
      %% Bilateral position constraints 
      if nP > 0
        if (nargout<2)
          [phiP,JP] = geval(@positionConstraints,obj.manip,q);
          %        [phiP,JP] = obj.manip.positionConstraints(q);
        else
          [phiP,JP,dJP] = geval(@positionConstraints,obj.manip,q);
          dJP(nL+(1:nP),:) = [dJP; -dJP];
        end
        phiP = [phiP;-phiP];
        JP = [JP; -JP];
        J(nL+(1:nP),:) = JP; 
      end
      
      %% Bilateral velocity constraints
      if nV > 0
        error('not implemented yet');  % but shouldn't be hard
      end
      
      M = zeros(nL+nP+(mC+2)*nC)*q(1);
      w = zeros(nL+nP+(mC+2)*nC,1)*q(1);
      active = repmat(true,nL+nP+(mC+2)*nC,1);
      active_tol = .01;
      
      % note: I'm inverting H twice here.  Should i do it only once, in a
      % previous step?
      Hinv = inv(H);
      wqdn = qd + h*Hinv*tau;
      Mqdn = Hinv*J';

      if (nargout>1)
        dM = zeros(size(M,1),size(M,2),1+2*num_q+obj.num_u);
        dw = zeros(size(w,1),1+2*num_q+obj.num_u);
        dwqdn = [zeros(num_q,1+num_q),eye(num_q),zeros(num_q,obj.num_u)] + ...
          h*Hinv*dtau - [zeros(num_q,1),h*Hinv*matGradMult(dH,Hinv*tau),zeros(num_q,obj.num_u)];
        dJtranspose = reshape(permute(reshape(dJ,size(J,1),size(J,2),[]),[2,1,3]),prod(size(J)),[]);
        dMqdn = [zeros(prod(size(Mqdn)),1),reshape(Hinv*reshape(dJtranspose - matGradMult(dH(:,1:num_q),Hinv*J'),num_q,[]),prod(size(Mqdn)),[]),zeros(prod(size(Mqdn)),num_q+obj.num_u)];
      end
      
      % check gradients
%      xdn = Mqdn;
%      if (nargout>1)
%        df = dMqdn;
%        df = [zeros(prod(size(xdn)),1),reshape(dJ,prod(size(xdn)),[]),zeros(prod(size(xdn)),num_q+obj.num_u)];
%      end
%      return;
      
      %% Joint Limits:
      % phiL(qn) is distance from each limit (in joint space)
      % phiL_i(qn) >= 0, cL_i >=0, phiL_i(qn) * cL_I = 0
      % z(1:nL) = cL (nL includes min AND max; 0<=nL<=2*num_q)
      % s(1:nL) = phiL(qn) approx phiL + h*JL*qdn
      if (nL > 0)
        w(1:nL) = phiL + h*JL*wqdn;
        M(1:nL,:) = h*JL*Mqdn;
        active(1:nL) = (phiL + h*JL*qd) < active_tol;
        if (nargout>1)
          dJL = [zeros(prod(size(JL)),1),reshape(dJL,prod(size(JL)),[]),zeros(prod(size(JL)),num_q+obj.num_u)];
          dw(1:nL,:) = [zeros(size(JL,1),1),JL,zeros(size(JL,1),num_q+obj.num_u)] + h*matGradMultMat(JL,wqdn,dJL,dwqdn);
          dM(1:nL,1:size(Mqdn,2),:) = reshape(h*matGradMultMat(JL,Mqdn,dJL,dMqdn),nL,size(Mqdn,2),[]);  
        end
      end
      
      %% Bilateral Position Constraints:
      % enforcing eq7, line 2
      if (nP > 0)
        w(nL+(1:nP)) = phiP + h*JP*wqdn;
        M(nL+(1:nP),:) = h*JP*Mqdn;
        active(nL+(1:nP)) = true;
        if (nargout>1)
          dw(nL+(1:nP),:) = JP + h*matGradMultMat(JP,wqdn,dJP,dwqdn);
          dM(nL+(1:nP),nL+(1:nP),:) = reshape(h*matGradMultMat(JP,Mqdn,dJP,qMqdn),nP,nP,[]);
        end
      end
      
      %% Contact Forces:
      % s(nL+nP+(1:nC)) = phiC+h*n*qdn  (modified (fixed?) from eq7, line 3)
      % z(nL+nP+(1:nC)) = cN
      % s(nL+nP+nC+(1:mC*nC)) = repmat(lambda,mC,1) + D*qdn  (eq7, line 4)
      % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
      % s(nL+nP+(mC+1)*nC+(1:nC)) = mu*cn - sum_mC beta_mC (eq7, line 5)
      % z(nL+nP+(mC+1)*nC+(1:nC)) = lambda
      if (nC > 0)
        w(nL+nP+(1:nC)) = phiC+h*n*wqdn;
        M(nL+nP+(1:nC),:) = h*n*Mqdn;
        
        w(nL+nP+nC+(1:mC*nC)) = D*wqdn;
        M(nL+nP+nC+(1:mC*nC),:) = D*Mqdn; 
        M(nL+nP+nC+(1:mC*nC),nL+nP+(1+mC)*nC+(1:nC)) = repmat(eye(nC),mC,1);

        M(nL+nP+(mC+1)*nC+(1:nC),nL+nP+(1:(mC+1)*nC)) = [diag(mu), repmat(-eye(nC),1,mC)];

        if (nargout>1)
          % n, dn, and dD were only w/ respect to q.  filled them out for [t,x,u]
          dn = [zeros(size(dn,1),1),dn,zeros(size(dn,1),num_q+obj.num_u)];
          dD = [zeros(prod(size(D)),1),reshape(dD,prod(size(D)),[]),zeros(prod(size(D)),num_q+obj.num_u)];
          
          dw(nL+nP+(1:nC),:) = [zeros(size(n,1),1),n,zeros(size(n,1),num_q+obj.num_u)]+h*matGradMultMat(n,wqdn,dn,dwqdn);
          dM(nL+nP+(1:nC),1:size(Mqdn,2),:) = reshape(h*matGradMultMat(n,Mqdn,dn,dMqdn),nC,size(Mqdn,2),[]);
          
          dw(nL+nP+nC+(1:mC*nC),:) = matGradMultMat(D,wqdn,dD,dwqdn);
          dM(nL+nP+nC+(1:mC*nC),1:size(Mqdn,2),:) = reshape(matGradMultMat(D,Mqdn,dD,dMqdn),mC*nC,size(Mqdn,2),[]);
        end
        
        a = (phiC+h*n*qd) < active_tol;
        active(nL+nP+(1:(mC+2)*nC),:) = repmat(a,mC+2,1);
      end
      
      % check gradients
%      xdn = M;
%      if (nargout>1)
%        df = reshape(dM,prod(size(M)),[]);
%      end
%      return;
      
      
      while (1)
        z = zeros(nL+nP+(mC+2)*nC,1);
        if any(active)
          z(active) = pathlcp(M(active,active),w(active));
        end
        
        inactive = ~active(1:(nL+nP+nC));  % only worry about the constraints that really matter.
        missed = (M(inactive,inactive)*z(inactive)+w(inactive) < 0);
        if ~any(missed), break; end
        % otherwise add the missed indices to the active set and repeat
        disp(['t=',num2str(t),': missed ',num2str(sum(missed)),' constraints.  resolving lcp.']);
        ind = find(inactive);
        inactive(ind(missed)) = false;
        % add back in the related contact terms:
        inactive = [inactive; repmat(inactive(nL+nP+(1:nC)),mC+1,1)];
        active = ~inactive;
      end 
      
      % for debugging
      %cN = z(nL+nP+(1:nC))
      %beta1 = z(nL+nP+nC+(1:nC))
      %beta2 = z(nL+nP+2*nC+(1:nC))
      %lambda = z(nL+nP+3*nC+(1:nC))
      % end debugging
      
      qdn = Mqdn*z + wqdn;
      qn = q + h*qdn;
      xdn = [qn;qdn];
      
      if (nargout>1)  % compute gradients
        % Quick derivation:
        % The LCP solves for z given that:
        % M(a)*z + q(a) >= 0
        % z >= 0
        % z'*(M(a)*z + q(a)) = 0
        % where the vector inequalities are element-wise, and 'a' is a vector of  parameters (here the state x and control input u).
        %
        % Our goal is to solve for the gradients dz/da.
        %
        % First we solve the LCP to obtain z.
        %
        % Then, for all i where z_i = 0, then dz_i / da = 0.
        % Call the remaining active constraints (where z_i >0)  Mbar(a), zbar, and  qbar(a).  then we have
        % Mbar(a) * zbar + qbar(a) = 0
        %
        % and the remaining gradients are given by
        % for all j, dMbar/da_j * zbar + Mbar * dzbar / da_j + dqbar / da_j = 0
        % or
        %
        % dzbar / da_j =  - inv(Mbar)*(dMbar/da_j * zbar + dqbar / da_j)
        %
        % I'm pretty sure that Mbar will always be invertible when the LCP is solvable.
        
        dz = zeros(size(z,1),1+obj.num_x+obj.num_u);
        zposind = find(z>0);
        if ~isempty(zposind)
          Mbar = M(zposind,zposind);
          dMbar = reshape(dM(zposind,zposind,:),prod(size(Mbar)),[]);
          zbar = z(zposind);
          dwbar = dw(zposind,:);
          dz(zposind,:) = -Mbar\(matGradMult(dMbar,zbar) + dwbar);
        end
        
        dqdn = matGradMult(dMqdn,z) + Mqdn*dz + dwqdn;
        df = [ [zeros(num_q,1), eye(num_q), zeros(num_q,num_q+obj.num_u)]+h*dqdn; dqdn ]; 
      end
      
    end

    function y = output(obj,t,x,u)
      if isDirectFeedthrough(obj)
        y = output(obj.manip,t,x,u);
      else
        y = output(obj.manip,t,x);
      end
    end

    function phi = stateConstraints(obj,x)
      phi = stateConstraints(obj.manip,x);
    end
    
    function v = constructVisualizer(obj)
      v = constructVisualizer(obj.manip);
    end

    function [xstar,ustar,success] = findFixedPoint(obj,x0,u0,v)
      % attempts to find a fixed point (xstar,ustar) which also satisfies the constraints,
      % using (x0,u0) as the initial guess.
      %
      % @param x0 initial guess for the state
      % @param u0 initial guess for the input
      % @param v (optional) a visualizer that should be called while the
      % solver is doing it's thing      
      
      if ~isTI(obj) error('only makes sense for time invariant systems'); end
            
      problem.objective = @(xu) 0;  % feasibility problem.   empty objective
      problem.x0 = [x0;u0];
      
      function [c,ceq,GC,GCeq] = mycon(xu)
        num_q = obj.manip.num_q;
        x = xu(1:obj.num_x);
        q = x(1:num_q); qd = x(num_q + (1:num_q));
        u = xu(obj.num_x + (1:obj.num_u));

        [phiL,JL] = jointLimits(obj.manip,q);
        [phiC,JC] = contactConstraints(obj.manip,q);
        c = -[phiL;phiC]; 
        GC = -[[JL,zeros(length(phiL),num_q+obj.num_u)]',[JC,zeros(length(phiC),num_q+obj.num_u)]'];
        
        if (obj.num_xcon) error('need to implement gradients for state constraints'); end

        ceq=[]; GCeq=[];
        if (num_q>0)
          [xdn,df] = update(obj,0,x,u);
          ceq=[ceq;x-xdn]; GCeq=[GCeq,([eye(obj.num_x),zeros(obj.num_x,obj.num_u)]-df(:,2:end))'];
        end
      end
      problem.nonlcon = @mycon;
      problem.solver = 'fmincon';

      function stop=drawme(xu,optimValues,state)
        stop=false;
        v.draw(0,xu(1:obj.num_x));
      end
      if (nargin>2 && ~isempty(v))  % useful for debugging (only but only works for URDF manipulators)
        problem.options=optimset('GradConstr','on','Algorithm','active-set','Display','iter','OutputFcn',@drawme,'TolX',1e-9);
      else
        problem.options=optimset('GradConstr','on','Algorithm','active-set','Display','off');
      end
      [xu,~,exitflag] = fmincon(problem);
      xstar = xu(1:obj.num_x);
      ustar = xu(obj.num_x + (1:obj.num_u));
      success=(exitflag==1);
      if (nargout<2 && ~success)
        error('Drake:PlanarRigidBodyManipulator:ResolveConstraintsFailed','failed to resolve constraints');
      end      
    end    
    
  end
  
  
end
  