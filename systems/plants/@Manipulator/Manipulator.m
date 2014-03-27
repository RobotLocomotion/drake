classdef Manipulator < DrakeSystem
% An abstract class that wraps H(q)vdot + C(q,v,f_ext) = B(q)u.

  methods
    function obj = Manipulator(num_q, num_u, num_v)
      if nargin<3, num_v = num_q; end
      
      obj = obj@DrakeSystem(num_q+num_v,0,num_u,num_q+num_v,false,true);
      obj.num_positions = num_q;
      obj.num_velocities = num_v;
    end
  end
  
  methods (Abstract=true)
    %  H(q)vdot + C(q,v,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,v);
%    [phat,estimated_delay] = parameterEstimation(obj,data,options)
  end

  methods 
    function [xdot,dxdot] = dynamics(obj,t,x,u)
    % Provides the DrakeSystem interface to the manipulatorDynamics.

      q = x(1:obj.num_positions);
      v = x(obj.num_positions+1:end);
    
      if (nargout>1)
        if (obj.num_xcon>0)
          % by naming this 'MATLAB:TooManyOutputs', geval will catch the
          % error and use TaylorVarInstead
          error('MATLAB:TooManyOutputs','User gradients for constrained dynamics not implemented yet.');
        end
        
        % Note: the next line assumes that user gradients are implemented.
        % If it fails, then it will raise the same exception that I would
        % want to raise for this method, stating that not all outputs were
        % assigned.  (since I can't write dxdot anymore)
        [H,C,B,dH,dC,dB] = obj.manipulatorDynamics(q,v);
        Hinv = inv(H);
        
        if (obj.num_u>0) 
          vdot = Hinv*(B*u-C); 
          dvdot = [zeros(obj.num_velocities,1),...
            Hinv*(matGradMult(dB(:,1:obj.num_positions),u) - matGradMult(dH(:,1:obj.num_positions),vdot) - dC(:,1:obj.num_positions)),...
            Hinv*(matGradMult(dB(:,obj.num_positions+1:end),u) - dC(:,obj.num_positions+1:end)), Hinv*B];
        else
          vdot = -Hinv*C; 
          dvdot = [zeros(obj.num_velocities,1),...
            Hinv*(-matGradMult(dH(:,1:obj.num_positions),vdot) - dC(:,1:obj.num_positions)),...
            Hinv*(-dC(:,obj.num_positions+1:end))];
        end
        
        [VqInv,dVqInv] = vToqdot(obj,q);
        xdot = [VqInv*v;vdot];
        dxdot = [zeros(obj.num_positions,1),dVqInv*v,VqInv,zeros(obj.num_positions,obj.num_u);dvdot];
      else
        [H,C,B] = manipulatorDynamics(obj,q,v);
        Hinv = inv(H);
        if (obj.num_u>0) tau=B*u - C; else tau=-C; end
        tau = tau + computeConstraintForce(obj,q,v,H,tau,Hinv);
      
        vdot = Hinv*tau;
        % note that I used to do this (instead of calling inv(H)):
        %   vdot = H\tau
        % but I already have and use Hinv, so use it again here
        
        xdot = [vToqdot(obj,q)*v; vdot];
      end      
      
    end
        
    function [Vq,dVq] = qdotTov(obj, q)
      % defines the linear map v = Vq * qdot
      % default relationship is that v = qdot
      assert(obj.num_positions==obj.num_velocities);
      Vq = eye(length(q));
      dVq = zeros(length(q));
    end
    
    function [VqInv,dVqInv] = vToqdot(obj, q)
      % defines the linear map qdot = Vqinv * v
      % default relationship is that v = qdot
      assert(obj.num_positions==obj.num_velocities);
      VqInv = eye(length(q));
      dVq = zeros(length(q));
    end
    
    function y = output(obj,t,x,u)
      % default output is the full state
      y = x;
    end
        
  end
  
  methods (Access=private)
    function constraint_force = computeConstraintForce(obj,q,v,H,tau,Hinv)
      % Helper function to compute the internal forces required to enforce 
      % equality constraints
      
      alpha = 10;  % 1/time constant of position constraint satisfaction (see my latex rigid body notes)
      beta = 0;    % 1/time constant of velocity constraint satisfaction
    
      phi=[]; psi=[];
      if (obj.num_position_constraints>0 && obj.num_velocity_constraints>0)
        [phi,Jv,Jvdot_times_v] = positionConstraintsV(obj,q,v);
        A = vToqdot(q);

        [psi,dpsi] = geval(@obj.velocityConstraints,q,v);
        dpsidq = dpsi(:,1:obj.num_positions);
        dpsidv = dpsi(:,obj.num_positions+1:end);
        
        term1=Hinv*[Jv;dpsidv]';
        term2=Hinv*tau;
        
        constraint_force = -[Jv;dpsidv]'*pinv([Jv*term1;dpsidv*term1])*[Jv*term2 + Jvdot_times_v + alpha*Jv*v; dpsidv*term2 + dpsidq*A*v + beta*psi];
      elseif (obj.num_position_constraints>0)  % note: it didn't work to just have dpsidq,etc=[], so it seems like the best solution is to handle each case...
        [phi,Jv,Jvdot_times_v] = positionConstraintsV(obj,q,v);

        constraint_force = -Jv'*pinv(Jv*Hinv*Jv')*(Jv*Hinv*tau + Jvdot_times_v + alpha*Jv*v);
      elseif (obj.num_velocity_constraints>0)
        [psi,Jv] = geval(@obj.velocityConstraints,q,v);
        A = vToqdot(q);
        dpsidq = J(:,1:obj.num_positions);
        dpsidv = J(:,obj.num_positions+1:end);
        
        constraint_force = -dpsidv'*pinv(dpsidv*Hinv*dpsidv')*(dpsidq*A*v + dpsidv*Hinv*tau + beta*psi);
      else
        constraint_force = 0*q;
      end
    end
  end
  
  methods
    function obj = setNumPositionConstraints(obj,num)
    % Set the number of bilateral constraints
      if (~isscalar(num) || num <0)
        error('num_bilateral_constraints must be a non-negative scalar');
      end
      obj.num_position_constraints=num;
      obj.num_xcon=2*obj.num_position_constraints+obj.num_velocity_constraints;
    end
    
    function obj = setNumVelocityConstraints(obj,num)
    % Set the number of bilateral constraints
      if (~isscalar(num) || num <0)
        error('num_bilateral_constraints must be a non-negative scalar');
      end
      obj.num_velocity_constraints=num;
      obj.num_xcon=2*obj.num_position_constraints+obj.num_velocity_constraints;
    end
  end
  
  methods (Sealed = true)
    function obj = setNumStateConstraints(obj,num)
      % Not a valid method.  Enforce that it is not called directly.
      error('you must set position constraints and velocity constraints explicitly.  cannot set general constraints for manipulator plants');
    end
  end
  
  methods
    function obj = setNumDOF(obj,num)
      error('setNumDOF is deprecated.  Use setNumPositions and setNumVelocities instead.');
    end
    function n = getNumDOF(obj)
      error('getNumDOF is deprecated.  In order to fully support quaternion floating base dynamics, we had to change the interface to allow a different number position and velocity elements in the state vector.  Use getNumPositions() and getNumVelocities() instead.');
    end
    
    function obj = setNumPositions(obj,num_q)
    % Guards the num_positions variable to make sure it stays consistent 
    % with num_x.
      obj.num_positions = num_q;
      obj = setNumContStates(obj,num_q+obj.num_velocities);
    end
    
    function n = getNumPositions(obj)
      n = obj.num_positions;
    end
    
    function obj = setNumVelocities(obj,num_v)
    % Guards the num_velocities variable to make sure it stays consistent 
    % with num_x.
      obj.num_velocities = num_v;
      obj = setNumContStates(obj,num_v+obj.num_positions);
    end
    
    function n = getNumVelocities(obj);
      n = obj.num_velocities;
    end
    
    function phi = positionConstraints(obj,q)
      % Implements position constraints of the form phi(q) = 0
      error('manipulators with position constraints must overload this function');
    end
    
    function [phi,Jv,Jvdot_times_v] = positionConstraintsV(obj,q,v)
      [phi,J,dJ] = geval(@obj.positionConstraints,q);
      A = vToqdot(q);
      qd = vToqdot(q)*v;
      Jv = J*A;
      Jv_times_v = dJ*reshape(qd*qd',obj.num_q^2,1);
    end
    
    function psi = velocityConstraints(obj,q,v)
      % Implements velocity constraints of the form psi(q,qdot) = 0
      % Note: dphidqdot must not be zero. constraints which depend 
      % only on q should be implemented instead as positionConstraints.
      error('manipulators with velocity constraints must overload this function'); 
    end
        
    function con = stateConstraints(obj,x)
      % wraps up the position and velocity constraints into the general constraint
      % method.  note that each position constraint (phi=0) also imposes an implicit
      % velocity constraint on the system (phidot=0).

      q=x(1:obj.num_positions); v=x(obj.num_positions+1:end);
      if (obj.num_position_constraints>0)
        [phi,Jv] = positionConstraintsV(obj,q,v);
      else
        phi=[]; Jv=zeros(0,obj.num_positions);
      end
      if (obj.num_velocity_constraints>0)
        psi = obj.velocityConstraints(q,v);
      else
        psi=[];
      end
      con = [phi; Jv*v; psi];  % phi=0, phidot=0, psi=0
    end
    
    function n = getNumJointLimitConstraints(obj)
      % returns number of constraints imposed by finite joint limits
      n = sum(obj.joint_limit_min ~= -inf) + sum(obj.joint_limit_max ~= inf);
    end
    
    function [phi,J,dJ] = jointLimitConstraints(obj,q)
      % constraint function (with derivatives) to implement unilateral
      % constraints imposed by joint limits
      phi = [q-obj.joint_limit_min; obj.joint_limit_max-q]; phi=phi(~isinf(phi));
      J = [eye(obj.num_positions); -eye(obj.num_positions)];  
      J([obj.joint_limit_min==-inf;obj.joint_limit_max==inf],:)=[]; 
      if (nargout>2)
        dJ = sparse(length(phi),obj.num_positions^2);
      end
    end
    
    function num_contacts(obj)
      error('num_contacts parameter is no longer supported, in anticipation of alowing multiple contacts per body pair. Use getNumContactPairs for cases where the number of contacts is fixed');
    end
    
    function n = getNumContacts(obj)
      error('getNumContacts is no longer supported, in anticipation of alowing multiple contacts per body pair. Use getNumContactPairs for cases where the number of contacts is fixed');
    end
    
    function [x,success] = resolveConstraints(obj,x0,visualizer)
      % attempts to find a x which satisfies the constraints,
      % using x0 as the initial guess.
      %
      % @param x0 initial guess for state satisfying constraints
      % @param visualizer (optional) a visualizer that should be called while the
      % solver is doing it's thing

      if isa(x0,'Point')
        x0 = double(x0.inFrame(obj.getStateFrame));
      end
      
      if (all(obj.joint_limit_min==-inf) && all(obj.joint_limit_max==inf) && obj.getNumUnilateralConstraints==0)
        if (nargin<3) visualizer=[]; end
        [x,success] = resolveConstraints@DrakeSystem(obj,x0,visualizer);
        return;
      end
      
      problem.objective = @(x) 0;  % feasibility problem.   empty objective
      problem.x0 = x0;
      
      function [c,ceq] = mycon(x)
        q = x(1:obj.num_positions);
        phi = obj.unilateralConstraints(x);
        c = -[jointLimitConstraints(obj,q); phi];
        ceq = stateConstraints(obj,x);
      end
      problem.nonlcon = @mycon;
      problem.solver = 'fmincon';

      function stop=drawme(x,optimValues,state)
        stop=false;
        visualizer.draw(0,x);
      end
      if (nargin>2 && ~isempty(visualizer))  % useful for debugging (only but only works for URDF manipulators)
        problem.options=optimset('Algorithm','active-set','Display','iter','OutputFcn',@drawme,'TolX',1e-9);
      else
        problem.options=optimset('Algorithm','active-set','Display','off');
      end
      [x,~,exitflag] = fmincon(problem);
      success=(exitflag==1);
      if (nargout<2 && ~success)
        error('Drake:Manipulator:ResolveConstraintsFailed','failed to resolve constraints');
      end
      x = Point(obj.getStateFrame,x);
    end
        
    function polysys = extractTrigPolySystem(obj,options)
      % Creates a (rational) polynomial system representation of the
      % dynamics
      
      if (obj.num_xcon>0) error('not implemented yet.  may not be possible.'); end
      
      function rhs = dynamics_rhs(obj,t,x,u)
        q=x(1:obj.num_positions); v=x((obj.num_positions+1):end);
        [~,C,B] = manipulatorDynamics(obj,q,v);
        if (obj.num_u>0) tau=B*u; else tau=zeros(obj.num_u,1); end
        rhs = [vToqdot(obj,q)*v;tau - C];
      end
      function lhs = dynamics_lhs(obj,x)
        q=x(1:obj.num_positions); v=x((obj.num_positions+1):end);
        H = manipulatorDynamics(obj,q,v);  % just get H
        lhs = blkdiag(eye(obj.num_positions),H);
      end        
      
      options.rational_dynamics_numerator=@(t,x,u)dynamics_rhs(obj,t,x,u);
      options.rational_dynamics_denominator=@(x)dynamics_lhs(obj,x);
      
      polysys = extractTrigPolySystem@DrakeSystem(obj,options);
    end

    function varargout = pdcontrol(sys,Kp,Kd,index)
      % creates new blocks to implement a PD controller, roughly
      % illustrated by
      %   q_d --->[ Kp ]-->(+)----->[ sys ]----------> yout
      %                     | -                 |
      %                     -------[ Kp,Kd ]<---- 
      %                       
      % when invoked with a single output argument:
      %   newsys = pdcontrol(sys,...)
      % then it returns a new system which contains the new closed loop
      % system containing the PD controller and the plant.
      %
      % when invoked with two output arguments:
      %   [pdff,pdfb] = pdcontrol(sys,...)
      % then it return the systems which define the feed-forward path and
      % feedback-path of the PD controller (but not the closed loop
      % system).
      %
      % @param Kp a num_u x num_u matrix with the position gains
      % @param Kd a num_u x num_u matrix with the velocity gains
      % @param index a num_u dimensional vector specifying the mapping from q to u.
      % index(i) = j indicates that u(i) actuates q(j). @default: 1:num_u
      %
      % For example, the a 2D floating base (with adds 3 passive joints in 
      % positions 1:3)model with four actuated joints in a serial chain might have 
      %      Kp = diag([10,10,10,10])
      %      Kd = diag([1, 1, 1, 1])
      %      and the default index would automatically be index = 4:7

      if nargin<4 || isempty(index)
        % try to extract the index from B
        q=msspoly('q',sys.num_q);
        s=msspoly('s',sys.num_q);
        c=msspoly('c',sys.num_q);
        qt=TrigPoly(q,s,c);
        qd=msspoly('v',sys.num_q);

        try 
          [~,~,B] = manipulatorDynamics(sys,qt,qd);
          B = double(B.getmsspoly);
          if ~isa(B,'double') error('B isn''t a constant'); end
          if ~all(sum(B~=0,2)==1) || ~all(sum(B~=0,1)==1)
            error('B isn''t simple.  Needs a single input to touch a single DOF.');
          end
          
          [I,J] = find(B);
          index(J)=I;
          
          % try to alert if it looks like there are any obvious sign errors
          if all(diag(diag(Kp))==Kp)
            d = diag(Kp);
            if any(sign(B(sub2ind(size(B),I,J)))~=sign(d(J)))
              warning('Drake:Manipulator:PDControlSignWarning','You might have a sign flipped?  The sign of Kp does not match the sign of the associated B');
            end
          end
          if all(diag(diag(Kd))==Kd)
            d = diag(Kd);
            if any(sign(B(sub2ind(size(B),I,J)))~=sign(d(J)))
              warning('Drake:Manipulator:PDControlSignWarning','You might have a sign flipped?  The sign of Kd does not match the sign of the associated B');
            end
          end
            
        catch  % because trigpolys aren't guaranteed to work for all manipulators
          warning('Drake:Manipulator:PDControlDefaultIndex','Couldn''t extract default index from the B matrix.  resorting to SecondOrderSystem default behavior.'); 
          warning(lasterr);
          index=[];
        end
      end
      
      varargout=cell(1,nargout);
      [varargout{:}] = pdcontrol@SecondOrderSystem(sys,Kp,Kd,index);
    end
    
    function [jl_min, jl_max] = getJointLimits(obj)
      % Returns lower and upper joint limit vectors
      jl_min = obj.joint_limit_min;
      jl_max = obj.joint_limit_max;
    end
  end  
  
  properties (SetAccess = protected, GetAccess = public)
    num_positions=0;
    num_velocities=0;
    num_position_constraints = 0  % the number of position constraints of the form phi(q)=0
    num_velocity_constraints = 0  % the number of velocity constraints of the form psi(q,qd)=0
    joint_limit_min = -inf;       % vector of length num_q with lower limits
    joint_limit_max = inf;        % vector of length num_q with upper limits
  end
end
