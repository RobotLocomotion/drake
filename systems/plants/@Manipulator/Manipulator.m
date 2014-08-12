classdef Manipulator < SecondOrderSystem
% An abstract class that wraps H(q)qddot + C(q,qdot,f_ext) = B(q)u.
% Coming soon:  will also support bilateral constraints of the form: phi(q)=0.

  methods
    function obj = Manipulator(num_q, num_u)
      % Constructor
      obj = obj@SecondOrderSystem(num_q,num_u,true);
      obj.joint_limit_min = -inf(num_q,1);
      obj.joint_limit_max = inf(num_q,1);
    end
  end
  
  methods (Abstract=true)
    %  H(q)qddot + C(q,qdot,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,qd);
  end

  methods (Sealed = true)
    % list this as Sealed to overcome the multiple inheritance diamond
    % problem: http://www.mathworks.com/help/matlab/matlab_oop/subclassing-multiple-classes.html
    [phat,estimated_delay] = parameterEstimation(obj,data,options)
      
    function [qdd,dqdd] = sodynamics(obj,t,q,qd,u)
    % Provides the SecondOrderDynamics interface to the manipulatorDynamics.

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
        [H,C,B,dH,dC,dB] = obj.manipulatorDynamics(q,qd);
        Hinv = inv(H);
        
        if (obj.num_u>0) 
          qdd = Hinv*(B*u-C); 
          dtau = matGradMult(dB,u) - dC;
          dqdd = [zeros(obj.num_q,1),...
            -Hinv*matGradMult(dH(:,1:obj.num_q),qdd) + Hinv*dtau(:,1:obj.num_q),...
            +Hinv*dtau(:,1+obj.num_q:end), Hinv*B];
        else
          qdd = Hinv*(-C);
          dqdd = [zeros(obj.num_q,1),...
            -Hinv*matGradMult(dH(:,1:obj.num_q),qdd) - Hinv*dC(:,1:obj.num_q),...
            -Hinv*dC(:,1+obj.num_q:end)];
        end
      else
        [H,C,B] = manipulatorDynamics(obj,q,qd);
        Hinv = inv(H);
        if (obj.num_u>0) tau=B*u - C; else tau=-C; end
        tau = tau + computeConstraintForce(obj,q,qd,H,tau,Hinv);
      
        qdd = Hinv*tau;
        % note that I used to do this (instead of calling inv(H)):
        %   qdd = H\tau
        % but I already have and use Hinv, so use it again here
      end      
      
    end
    
  end
  
  methods (Access=private)
    function constraint_force = computeConstraintForce(obj,q,qd,H,tau,Hinv)
      % Helper function to compute the internal forces required to enforce 
      % equality constraints
      
      alpha = 10;  % 1/time constant of position constraint satisfaction (see my latex rigid body notes)
      beta = 0;    % 1/time constant of velocity constraint satisfaction
    
      phi=[]; psi=[];
      if (obj.num_position_constraints>0 && obj.num_velocity_constraints>0)
        [phi,J,dJ] = geval(@obj.positionConstraints,q);
        Jdotqd = dJ*reshape(qd*qd',obj.num_q^2,1);

        [psi,dpsi] = geval(@obj.velocityConstraints,q,qd);
        dpsidq = dpsi(:,1:obj.num_q);
        dpsidqd = dpsi(:,obj.num_q+1:end);
        
        term1=Hinv*[J;dpsidqd]';
        term2=Hinv*tau;
        
        constraint_force = -[J;dpsidqd]'*pinv([J*term1;dpsidqd*term1])*[J*term2 + Jdotqd + alpha*J*qd; dpsidqd*term2 + dpsidq*qd + beta*psi];
      elseif (obj.num_position_constraints>0)  % note: it didn't work to just have dpsidq,etc=[], so it seems like the best solution is to handle each case...
        [phi,J,dJ] = geval(@obj.positionConstraints,q);
        Jdotqd = dJ*reshape(qd*qd',obj.num_q^2,1);

        constraint_force = -J'*pinv(J*Hinv*J')*(J*Hinv*tau + Jdotqd + alpha*J*qd);
      elseif (obj.num_velocity_constraints>0)
        [psi,J] = geval(@obj.velocityConstraints,q,qd);
        dpsidq = J(:,1:obj.num_q);
        dpsidqd = J(:,obj.num_q+1:end);
        
        constraint_force = -dpsidqd'*pinv(dpsidqd*Hinv*dpsidqd')*(dpsidq*qd + dpsidqd*Hinv*tau+beta*psi);
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
    function phi = positionConstraints(obj,q)
      % Implements position constraints of the form phi(q) = 0
      error('manipulators with position constraints must overload this function');
    end
    
    function psi = velocityConstraints(obj,q,qd)
      % Implements velocity constraints of the form psi(q,qdot) = 0
      % Note: dphidqdot must not be zero. constraints which depend 
      % only on q should be implemented instead as positionConstraints.
      error('manipulators with velocity constraints must overload this function'); 
    end
    
    function [con,dcon] = stateConstraints(obj,x)
      % wraps up the position and velocity constraints into the general constriant
      % method.  note that each position constraint (phi=0) also imposes an implicit
      % velocity constraint on the system (phidot=0).

      q=x(1:obj.num_q); qd=x(obj.num_q+1:end);
      if (obj.num_position_constraints>0)
        if (nargout>1)
          [phi,J,dJ] = geval(@obj.positionConstraints,q);
        else
          [phi,J] = geval(@obj.positionConstraints,q);
        end
      else
        phi=[]; J=zeros(0,obj.num_q); dJ=zeros(0,obj.num_q^2);
      end
      if (obj.num_velocity_constraints>0)
        if (nargout>1)
          [psi,dpsi] = obj.velocityConstraints(q,qd);
        else
          psi = obj.velocityConstraints(q,qd);
        end
      else
        psi=[]; dpsi=zeros(0,obj.num_x);
      end
        
      con = [phi; J*qd; psi];  % phi=0, phidot=0, psi=0
      if (nargout>1)
        dcon = [J,0*J; matGradMult(reshape(dJ,size(dJ,1)*obj.num_q,obj.num_q),qd), J; dpsi];
      end
    end
    
    function n = getNumJointLimitConstraints(obj)
      % returns number of constraints imposed by finite joint limits
      n = sum(obj.joint_limit_min ~= -inf) + sum(obj.joint_limit_max ~= inf);
    end
    
    function [phi,J,dJ] = jointLimitConstraints(obj,q)
      % constraint function (with derivatives) to implement unilateral
      % constraints imposed by joint limits
      phi = [q-obj.joint_limit_min; obj.joint_limit_max-q]; phi=phi(~isinf(phi));
      J = [eye(obj.num_q); -eye(obj.num_q)];  
      J([obj.joint_limit_min==-inf;obj.joint_limit_max==inf],:)=[]; 
      if (nargout>2)
        dJ = sparse(length(phi),obj.num_q^2);
      end
    end
    
    function num_contacts(obj)
      error('num_contacts parameter is no longer supported, in anticipation of alowing multiple contacts per body pair. Use getNumContactPairs for cases where the number of contacts is fixed');
    end
    
    function n = getNumContacts(obj)
      error('getNumContacts is no longer supported, in anticipation of alowing multiple contacts per body pair. Use getNumContactPairs for cases where the number of contacts is fixed');
    end
    
    function prog = addStateConstraintsToProgram(obj,prog,indices)
      % adds state constraints and unilateral constriants to the 
      %   program on the specified indices.  derived classes can overload 
      %   this method to add additional constraints.
      % 
      % @param prog a NonlinearProgramWConstraintObjects class
      % @param indices the indices of the state variables in the program
      %        @default 1:nX

      if nargin<3, indices=1:obj.num_x; end
      prog = addStateConstraintsToProgram@SecondOrderSystem(obj,prog,indices);
      
      % add joint limit constraints
      prog = addConstraint(prog,BoundingBoxConstraint(obj.joint_limit_min,obj.joint_limit_max),1:obj.num_q);
    end
    
    function sys = feedback(sys1,sys2)
      % Attempt to produce a new manipulator system if possible
      
      if (isa(sys2,'Manipulator'))
        % todo: implement this (or decide that it doesn't ever make sense)
        warning('feedback combinations of manipulators not handled explicitly yet. kicking out to a combination of SecondOrderSystems');
      end
      sys = feedback@SecondOrderSystem(sys1,sys2);
    end
    
    function sys = cascade(sys1,sys2)
      % Attempt to produce a new manipulator system if possible

      if (isa(sys2,'Manipulator'))
        % todo: implement this (or decide that it doesn't ever make sense)
        warning('cascade combinations of manipulators not handled explicitly yet. kicking out to a combination of SecondOrderSystems');
      end
      sys = cascade@SecondOrderSystem(sys1,sys2);
    end
    
    function polysys = extractTrigPolySystem(obj,options)
      % Creates a (rational) polynomial system representation of the
      % dynamics
      
      if (obj.num_xcon>0) error('not implemented yet.  may not be possible.'); end
      
      function rhs = dynamics_rhs(obj,t,x,u)
        q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
        [H,C,B] = manipulatorDynamics(obj,q,qd);
        if (obj.num_u>0) tau=B*u; else tau=zeros(obj.num_q,1); end
        rhs = [qd;tau - C];
      end
      function lhs = dynamics_lhs(obj,x)
        q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
        H = manipulatorDynamics(obj,q,qd);  % just get H
        lhs = blkdiag(eye(obj.num_q),H);
      end        
      
      options.rational_dynamics_numerator=@(t,x,u)dynamics_rhs(obj,t,x,u);
      options.rational_dynamics_denominator=@(x)dynamics_lhs(obj,x);
      
      polysys = extractTrigPolySystem@SecondOrderSystem(obj,options);
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
    
    function [lb,ub] = getStateLimits(obj)
      % Returns lower and upper state vectors. Uses joint limits for
      % positions and +/-inf for velocities
      [jl_min, jl_max] = getJointLimits(obj);
      lb = [jl_min; -inf(obj.getNumVelocities,1)];
      ub = [jl_max; inf(obj.getNumVelocities,1)];
    end
  end  
  
  properties (SetAccess = protected, GetAccess = public)
    num_position_constraints = 0  % the number of position constraints of the form phi(q)=0
    num_velocity_constraints = 0  % the number of velocity constraints of the form psi(q,qd)=0
    joint_limit_min = -inf;       % vector of length num_q with lower limits
    joint_limit_max = inf;        % vector of length num_q with upper limits
  end
end
