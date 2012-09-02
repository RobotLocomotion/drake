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
    
    function xdn = update(obj,t,x,u)
      % do LCP time-stepping
      num_q = obj.manip.num_q;
      q=x(1:num_q); qd=x((num_q+1):end);
      if (obj.twoD) d=2; else d=3; end 
      h = obj.timestep;

      [H,C,B] = manipulatorDynamics(obj.manip,q,qd);
      if (obj.num_u>0) tau = B*u - C; else tau = -C; end
      
      nL = sum([obj.manip.joint_limit_min~=-inf;obj.manip.joint_limit_max~=inf]); % number of joint limits
      nC = obj.manip.num_contacts;
      nP = d*obj.manip.num_position_constraints;  % number of position constraints
      nV = d*obj.manip.num_velocity_constraints;  
      
      if (nC+nL+nP+nV==0)
        qd_out = qd + h*H\tau;
        q_out = q + h*qd_out;
        xdn = [q_out; qd_out];
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
        [phiC,n,D,mu] = obj.manip.contactConstraints(q);
        mC = length(D);
        J = zeros(nL + nP + (mC+2)*nC,num_q);
        D = vertcat(D{:});
        J(nL+nP+(1:nC),:) = n;
        J(nL+nP+nC+(1:mC*nC),:) = D;
      else
        mC=0;
        J = zeros(nL+nP,num_q);
      end
      
      if (nL > 0)
        [phiL,JL] = obj.manip.jointLimits(q);
        J(1:nL,:) = JL;
      end
      
      %% Bilateral position constraints 
      if nP > 0
        [phiP,JP] = geval(@positionConstraints,obj.manip,q);
%        [phiP,JP] = obj.manip.positionConstraints(q);
        phiP = [phiP;-phiP];
        JP = [JP; -JP];
        J(nL+(1:nP),:) = JP; 
      end
      
      %% Bilateral velocity constraints
      if nV > 0
        error('not implemented yet');  % but shouldn't be hard
      end
      
      M = zeros(nL+nP+(mC+2)*nC);
      w = zeros(nL+nP+(mC+2)*nC,1);
      active = repmat(true,nL+nP+(mC+2)*nC,1);
      active_tol = .01;
      
      % note: I'm inverting H twice here.  Should i do it only once, in a
      % previous step?
      wqdn = qd + h*(H\tau);
      Mqdn = H\J';
      
      %% Joint Limits:
      % phiL(qn) is distance from each limit (in joint space)
      % phiL_i(qn) >= 0, cL_i >=0, phiL_i(qn) * cL_I = 0
      % z(1:nL) = cL (nL includes min AND max; 0<=nL<=2*num_q)
      % s(1:nL) = phiL(qn) approx phiL + h*JL*qdn
      if (nL > 0)
        w(1:nL) = phiL + h*JL*wqdn;
        M(1:nL,:) = h*JL*Mqdn;
        active(1:nL) = (phiL + h*JL*qd) < active_tol;
      end
      
      %% Bilateral Position Constraints:
      % enforcing eq7, line 2
      if (nP > 0)
        w(nL+(1:nP)) = phiP + h*JP*wqdn;
        M(nL+(1:nP),:) = h*JP*Mqdn;
        active(nL+(1:nP)) = true;
      end
      
      %% Contact Forces:
      % s(nL+nP+(1:nC)) = phiC+n*qdn  (modified (fixed?) from eq7, line 3)
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

        a = (phiC+h*n*qd) < active_tol;
        active(nL+nP+(1:(mC+2)*nC),:) = repmat(a,mC+2,1);
      end
      
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

  end
  
  
end
  