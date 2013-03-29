classdef TimeSteppingRigidBodyManipulator < DrakeSystem
  % A discrete time system which simulates (an Euler approximation of) the
  % manipulator equations, with contact / limits resolved using the linear
  % complementarity problem formulation of contact in Stewart96.
  
  properties (Access=protected)
    manip  % the CT manipulator
    sensor % additional TimeSteppingRigidBodySensors (beyond the sensors attached to manip)
    dirty=true;
  end

  properties (SetAccess=protected)
    timestep
    twoD=false
    position_control=false;
  end
  
  methods
    function obj=TimeSteppingRigidBodyManipulator(manipulator_or_urdf_filename,timestep,options)
      checkDependency('pathlcp_enabled');
      if (nargin<3) options=struct(); end
      if ~isfield(options,'twoD') options.twoD = false; end
      
      typecheck(timestep,'double');
      sizecheck(timestep,1);
      
      if isempty(manipulator_or_urdf_filename) || ischar(manipulator_or_urdf_filename)
        if options.twoD
          S = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits');
          warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedContactPoints');
          warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
          manip = PlanarRigidBodyManipulator(manipulator_or_urdf_filename,options);
          warning(S);
        else
          % then make the corresponding manipulator
          S = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
          warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
          manip = RigidBodyManipulator(manipulator_or_urdf_filename,options);
          warning(S);
        end
      else
        manip = manipulator_or_urdf_filename;
      end
      typecheck(manip,'RigidBodyManipulator');
      obj = obj@DrakeSystem(0,manip.getNumStates(),manip.getNumInputs(),manip.getNumOutputs(),manip.isDirectFeedthrough(),manip.isTI());
      obj.manip = manip;
      if isa(manip,'PlanarRigidBodyManipulator')
        obj.twoD = true;
      end
      
      obj.timestep = timestep;
      
      obj = setSampleTime(obj,[timestep;0]);
            
      obj = compile(obj);
    end
    
    function checkDirty(obj)
      if (obj.dirty)
        error('You''ve changed something about this model and need to manually compile it.  Use obj=compile(obj).');
      end
    end
    
    function y = output(obj,t,x,u)
      checkDirty(obj);
      
      if ~isDirectFeedthrough(obj)
        u=[];
      end
      
      y = obj.manip.output(t,x,u);
      for i=1:length(obj.sensor)
        y = [y; obj.sensor{i}.output(obj,obj.manip,t,x,u)];
      end
    end

    function model = enableIdealizedPositionControl(model, flag)
      index = getActuatedJoints(model.manip);
      if length(unique(index))~=length(index)
        error('idealized position control currently assumes one actuator per joint'); 
      end
      model.position_control = logical(flag);
      model.dirty = true;
    end
    
    function model = compile(model)
      if model.twoD
        S = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits');
        warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedContactPoints');
        warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      else
        S = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
        warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      end        
      model.manip = model.manip.compile();
      warning(S);
      
      model = setNumDiscStates(model,model.manip.getNumContStates());
      model = setNumInputs(model,model.manip.getNumInputs());
      
      if (model.position_control)
        index = getActuatedJoints(model.manip);
        pdff = pdcontrol(model,eye(model.num_u),eye(model.num_u));
        model = setInputLimits(model,model.manip.joint_limit_min(index),model.manip.joint_limit_max(index));
        model = setInputFrame(model,getInputFrame(pdff));
      else
        model = setInputLimits(model,model.manip.umin,model.manip.umax);
        model = setInputFrame(model,getInputFrame(model.manip));
      end
      model = setStateFrame(model,getStateFrame(model.manip));

      if length(model.sensor)>0
        feedthrough = model.manip.isDirectFeedthrough;
        outframe{1} = getOutputFrame(model.manip);
        for i=1:length(model.sensor)
          model.sensor{i} = model.sensor{i}.compile(model,model.manip);
          outframe{i+1} = model.sensor{i}.getFrame(model);
          feedthrough = feedthrough || model.sensor{i}.isDirectFeedthrough;
        end
        fr = MultiCoordinateFrame.constructFrame(outframe);
        model = setNumOutputs(model,fr.dim);
        model = setOutputFrame(model,fr);
        model = setDirectFeedthrough(model,feedthrough);
      else
        model = setNumOutputs(model,getNumOutputs(model.manip));
        model = setOutputFrame(model,getOutputFrame(model.manip));
        model = setDirectFeedthrough(model,model.manip.isDirectFeedthrough);
      end
      model.dirty = false;
    end
    
    function x0 = getInitialState(obj)
      x0 = obj.manip.getInitialState();
    end
    
    function B = getB(obj)
      B = obj.manip.getB();
    end
    
    function num_q = getNumDOF(obj)
      num_q = obj.manip.num_q;
    end
    
    function [xdn,df] = update(obj,t,x,u)
      if (nargout>1)
        [z,Mqdn,wqdn,dz,dMqdn,dwqdn] = solveLCP(obj,t,x,u);
      else
        [z,Mqdn,wqdn] = solveLCP(obj,t,x,u);
      end
      
      num_q = obj.manip.num_q;
      q=x(1:num_q); qd=x((num_q+1):end);
      h = obj.timestep;

      qdn = Mqdn*z + wqdn;
      qn = q + h*qdn;
      xdn = [qn;qdn];
      
      if (nargout>1)  % compute gradients
        warning('timestepping gradients don''t work for all cases.. see bug 1155');
        
        dqdn = matGradMult(dMqdn,z) + Mqdn*dz + dwqdn;
        df = [ [zeros(num_q,1), eye(num_q), zeros(num_q,num_q+obj.num_u)]+h*dqdn; dqdn ]; 
      end
    end
    
    function [z,Mqdn,wqdn,dz,dMqdn,dwqdn] = solveLCP(obj,t,x,u)
      % do LCP time-stepping
      
      % todo: implement some basic caching here
      
      num_q = obj.manip.num_q;
      q=x(1:num_q); qd=x((num_q+1):end);
      h = obj.timestep;

      if (nargout<4)
        [H,C,B] = manipulatorDynamics(obj.manip,q,qd);
        if (obj.num_u>0 && ~obj.position_control) tau = B*u - C; else tau = -C; end
      else
        [H,C,B,dH,dC,dB] = manipulatorDynamics(obj.manip,q,qd);
        if (obj.num_u>0 && ~obj.position_control) 
          tau = B*u - C;  
          dtau = [zeros(num_q,1), matGradMult(dB,u) - dC, B];
        else
          tau = -C; 
          dtau = [zeros(num_q,1), -dC];
        end
      end
      
      if (obj.position_control)
        pos_control_index = getActuatedJoints(obj.manip);
        nL = 2*length(pos_control_index);
      else
        nL = sum([obj.manip.joint_limit_min~=-inf;obj.manip.joint_limit_max~=inf]); % number of joint limits
      end
      nC = obj.manip.num_contacts;
      nP = 2*obj.manip.num_position_constraints;  % number of position constraints
      nV = obj.manip.num_velocity_constraints;  
      
      if (nC+nL+nP+nV==0)
        qd_out = qd + h*(H\tau);
        q_out = q + h*qd_out;
        xdn = [q_out; qd_out];
        if (nargout>3) error('need to implement this case'); end
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
        if (nargout>3)
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
        if (nargout>3)
          dJ = sparse(nL+nP,num_q^2);
        end
      end
      
      if (nL > 0)
        if (obj.position_control)
          phiL = q(pos_control_index) - u; 
          JL = sparse(1:obj.manip.num_u,pos_control_index,1,obj.manip.num_u,obj.manip.num_q);
          phiL = [phiL;-phiL]; JL = [JL;-JL];
          % dJ = 0 by default, which is correct here
        else
          if (nargout<4)
            [phiL,JL] = obj.manip.jointLimits(q);
          else
            [phiL,JL,dJL] = obj.manip.jointLimits(q);
            dJ(1:nL,:) = dJL;
          end
        end
        J(1:nL,:) = JL;
      end
      
      %% Bilateral position constraints 
      if nP > 0
        if (nargout<4)
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
      active = true(nL+nP+(mC+2)*nC,1);
      active_tol = .01;
      
      Hinv = inv(H);
      wqdn = qd + h*Hinv*tau;
      Mqdn = Hinv*J';

      if (nargout>3)
        dM = zeros(size(M,1),size(M,2),1+2*num_q+obj.num_u);
        dw = zeros(size(w,1),1+2*num_q+obj.num_u);
        dwqdn = [zeros(num_q,1+num_q),eye(num_q),zeros(num_q,obj.num_u)] + ...
          h*Hinv*dtau - [zeros(num_q,1),h*Hinv*matGradMult(dH(:,1:num_q),Hinv*tau),zeros(num_q,num_q),zeros(num_q,obj.num_u)];
        dJtranspose = reshape(permute(reshape(dJ,size(J,1),size(J,2),[]),[2,1,3]),prod(size(J)),[]);
        dMqdn = [zeros(numel(Mqdn),1),reshape(Hinv*reshape(dJtranspose - matGradMult(dH(:,1:num_q),Hinv*J'),num_q,[]),numel(Mqdn),[]),zeros(numel(Mqdn),num_q+obj.num_u)];
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
        if (nargout>3)
          dJL = [zeros(prod(size(JL)),1),reshape(dJL,numel(JL),[]),zeros(numel(JL),num_q+obj.num_u)];
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
        if (nargout>3)
          dJP = [zeros(numel(JP),1),reshape(dJP,numel(JP),[]),zeros(numel(JP),num_q+obj.num_u)];
          dw(nL+(1:nP),:) = [zeros(size(JP,1),1),JP,zeros(size(JP,1),num_q+obj.num_u)] + h*matGradMultMat(JP,wqdn,dJP,dwqdn);
          dM(nL+(1:nP),1:size(Mqdn,2),:) = reshape(h*matGradMultMat(JP,Mqdn,dJP,qMqdn),nP,size(Mqdn,2),[]);
        end
      end
      
      %% Contact Forces:
      % s(nL+nP+(1:nC)) = phiC+h*n*qdn  (modified (fixed?) from eq7, line 3)
      % z(nL+nP+(1:nC)) = cN
      % s(nL+nP+nC+(1:mC*nC)) = repmat(lambda,mC,1) + D*qdn  (eq7, line 4)
      % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
      % s(nL+nP+(mC+1)*nC+(1:nC)) = mu*cn - sum_mC beta_mC (eq7, line 5)
      % z(nL+nP+(mC+1)*nC+(1:nC)) = lambda
      %
      % The second set of conditions gives:
      %   lambda_i >= the largest projection of the velocity vector
      %   onto the d vectors (since lambda_i >= -(D*qdn)_i for all i, 
      % and by construction of d always having the mirror vectors,
      %   lambda_i >= (D_qdn)_i
      %
      % The last eqs give
      %  lambda_i > 0 iff (sum beta)_i = mu_i*cn_i  
      % where i is for the ith contact.
      % Assume for a moment that mu_i*cn_i = 1, then (sum_beta)_i = 1
      % is like a constraint ensuring that sum_beta_j D_j is like a convex
      % combination of the D vectors (since beta_j is also > 0)
      % So lambda_i >0 if forces for the ith contact are on the boundary of
      % the friction cone (lambda_i could also be > 0 if some of the beta_j
      % D_j's are internally canceling each other out)
      %
      % So one solution is 
      %  v_i = 0, 
      %  beta_i >= 0
      %  lambda_i = 0, 
      %  sum_d beta_i < mu*cn_i
      % and another solution is
      %  v_i > 0  (sliding) 
      %  lambda_i = max_d (v_i)
      %  beta_i = mu*cn_i only in the direction of the largest negative velocity
      %  beta_i = 0 otherwise
      % By virtue of the eqs of motion connecting v_i and beta_i, only one
      % of these two can exist. (the first is actually many solutions, with
      % beta_i in opposite directions canceling each other out).
      if (nC > 0)
        w(nL+nP+(1:nC)) = phiC+h*n*wqdn;
        M(nL+nP+(1:nC),:) = h*n*Mqdn;
        
        w(nL+nP+nC+(1:mC*nC)) = D*wqdn;
        M(nL+nP+nC+(1:mC*nC),:) = D*Mqdn; 
        M(nL+nP+nC+(1:mC*nC),nL+nP+(1+mC)*nC+(1:nC)) = repmat(eye(nC),mC,1);

        M(nL+nP+(mC+1)*nC+(1:nC),nL+nP+(1:(mC+1)*nC)) = [diag(mu), repmat(-eye(nC),1,mC)];

        if (nargout>3)
          % n, dn, and dD were only w/ respect to q.  filled them out for [t,x,u]
          dn = [zeros(size(dn,1),1),dn,zeros(size(dn,1),num_q+obj.num_u)];
          dD = [zeros(numel(D),1),reshape(dD,numel(D),[]),zeros(numel(D),num_q+obj.num_u)];
          
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
      
      if (nargout>3)
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
          dMbar = reshape(dM(zposind,zposind,:),numel(Mbar),[]);
          zbar = z(zposind);
          dwbar = dw(zposind,:);
          dz(zposind,:) = -Mbar\(matGradMult(dMbar,zbar) + dwbar);
        end
      end      
      
    end

    function obj = addSensor(obj,sensor)
      if isa(sensor,'RigidBodySensor')
        obj.manip = obj.manip.addSensor(sensor);
      else
        typecheck(sensor,'TimeSteppingRigidBodySensor');
        obj.sensor{end+1} = sensor;
      end
    end
    

    function varargout = pdcontrol(sys,Kp,Kd,index)
      if nargin<4, index=[]; end
      [pdff,pdfb] = pdcontrol(sys.manip,Kp,Kd,index);
      pdfb = setInputFrame(pdfb,sys.getStateFrame());
      pdfb = setOutputFrame(pdfb,sys.getInputFrame());
      pdff = setOutputFrame(pdff,sys.getInputFrame());
      if nargout>1
        varargout{1} = pdff;
        varargout{2} = pdfb;
      else
        % note: design the PD controller with the (non time-stepping
        % manipulator), but build the closed loop system with the
        % time-stepping manipulator:
        varargout{1} = cascade(pdff,feedback(sys,pdfb));
      end
    end
    
  end
  
  methods  % pass through methods (to the manipulator)
    function obj = setStateFrame(obj,fr)
      obj = setStateFrame@DrakeSystem(obj,fr);
      if ~isempty(obj.manip)  % this gets called in the constructor, before manip is ste
        obj.manip = setStateFrame(obj.manip,fr);
      end
    end
    
    function obj = setTerrain(obj,varargin)
      obj.manip = setTerrain(obj.manip,varargin{:});
    end
    
    function obj=addRobotFromURDF(obj,varargin)
      if obj.twoD
        S = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits');
        warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedContactPoints');
        warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      else
        S = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
        warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      end
      obj.manip=addRobotFromURDF(obj.manip,varargin{:});
      obj=compile(obj);  % note: compiles the manip twice, but it's ok.
      warning(S);
    end

    function varargout = doKinematics(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=doKinematics(obj.manip,varargin{:});
    end
    
    function varargout = forwardKin(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=forwardKin(obj.manip,varargin{:});
    end
    
    function varargout = bodyKin(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=bodyKin(obj.manip,varargin{:});
    end
    
    function varargout = inverseKin(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=inverseKin(obj.manip,varargin{:});
    end
    
    function varargout = inverseKinSequence(obj,varargin)
        varargout = cell(1,nargout);
        [varargout{:}] = inverseKinSequence(obj.manip,varargin{:});
    end
    
    function varargout = findFixedPoint(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=findFixedPoint(obj.manip,varargin{:});
    end
    
    function varargout = collisionDetect(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=collisionDetect(obj.manip,varargin{:});
    end
    
    function varargout = stateConstraints(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = stateConstraints(obj.manip,varargin{:});
    end
    
    function varargout = manipulatorDynamics(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = manipulatorDynamics(obj.manip,varargin{:});
    end
    
    function varargout = contactConstraints(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = contactConstraints(obj.manip,varargin{:});
    end
    
    function varargout = contactPositions(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = contactPositions(obj.manip,varargin{:});
    end
    
    function varargout = resolveConstraints(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = resolveConstraints(obj.manip,varargin{:});
      varargout{1} = Point(obj.getStateFrame,double(varargout{1}));
    end
    
    function varargout = getMass(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = getMass(obj.manip,varargin{:});
    end
    
    function varargout = getCOM(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = getCOM(obj.manip,varargin{:});
    end
    
    function grav = getGravity(obj)
      grav = obj.manip.gravity;
    end
    
    function body_ind = findLinkInd(model,varargin)
        body_ind = model.manip.findLinkInd(varargin{:});
    end
    
    function body = findLink(model,varargin)
      body = model.manip.findLink(varargin{:});
    end
    
    function v = constructVisualizer(obj)
      v = constructVisualizer(obj.manip);
      v = setInputFrame(v,obj.getStateFrame());
    end
    
    function num_c = getNumContacts(obj)
      num_c = obj.manip.num_contacts;
    end

    function c = getBodyContacts(obj,body_idx)
      c = obj.manip.body(body_idx).contact_pts;
    end
    
    function link_names = getLinkNames(obj)
      link_names =  {obj.manip.body.linkname}';
    end

    function joint_names = getJointNames(obj)
      joint_names =  {obj.manip.body.jointname}';
    end

    function num_bodies = getNumBodies(obj)
      num_bodies = length(obj.manip.body);
    end

    function [jl_min, jl_max] = getJointLimits(obj)
      jl_min = obj.manip.joint_limit_min;
      jl_max = obj.manip.joint_limit_max;
    end
  
    function index = getActuatedJoints(obj)
      index = obj.manip.getActuatedJoints();
    end
  end
  
  
end
  