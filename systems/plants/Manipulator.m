classdef Manipulator < SecondOrderSystem
% An abstract class that wraps H(q)qddot + C(q,qdot,f_ext) = B(q)u.
% Coming soon:  will also support bilateral constraints of the form: phi(q)=0.

  methods
    function obj = Manipulator(num_q, num_u, num_position_constraints, num_velocity_constraints)
      obj = obj@SecondOrderSystem(num_q,num_u,true);
      if (nargin>2)  % else num_position_constraints=0 by default
        obj = obj.setNumPositionConstraints(num_position_constraints);
      end
      if (nargin>3)  % else num_velocity_constraints=0 by default
        obj = obj.setNumVelocityConstraints(num_velocity_constraints);
      end
    end
  end
  
  methods (Abstract=true)
    %  H(q)qddot + C(q,qdot,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,qd);
  end

  methods
    function x0 = getInitialState(obj)
      x0 = randn(obj.num_xd+obj.num_xc,1);
      x0 = resolveConstraints(obj,x0);
    end

    function qdd = sodynamics(obj,t,q,qd,u)
    % Provides the SecondOrderDynamics interface to the manipulatorDynamics.

      alpha = 10;  % 1/time constant of position constraint satisfaction (see my latex rigid body notes)
      beta = 0;    % 1/time constant of velocity constraint satisfaction
    
      [H,C,B] = manipulatorDynamics(obj,q,qd);
      Hinv = inv(H);
      
      if (obj.num_u>0) tau=B*u; else tau=zeros(obj.num_q,1); end
      
      phi=[]; psi=[];
      if (obj.num_position_constraints>0 && obj.num_velocity_constraints>0)
        [phi,J,dJ] = geval(@obj.positionConstraints,q);
        Jdotqd = dJ*reshape(qd*qd',obj.num_q^2,1);

        [psi,dpsi] = geval(@obj.velocityConstraints,q,qd);
        dpsidq = dpsi(:,1:obj.num_q);
        dpsidqd = dpsi(:,obj.num_q+1:end);
        
        term1=Hinv*[J;dpsidqd]';
        term2=Hinv*(tau-C);
        
        constraint_force = -[J;dpsidqd]'*inv([J*term1;dpsidqd*term1])*[J*term2 + Jdotqd + alpha*J*qd; dpsidqd*term2 + dpsidq*qd + beta*psi];
      elseif (obj.num_position_constraints>0)  % note: it didn't work to just have dpsidq,etc=[], so it seems like the best solution is to handle each case...
        [phi,J,dJ] = geval(@obj.positionConstraints,q);
        Jdotqd = dJ*reshape(qd*qd',obj.num_q^2,1);

        constraint_force = -J'*(inv(J*Hinv*J')*(J*Hinv*(tau-C) + Jdotqd + alpha*J*qd));
      elseif (obj.num_velocity_constraints>0)
        [psi,J] = geval(@obj.velocityConstraints,q,qd);
        dpsidq = J(:,1:obj.num_q);
        dpsidqd = J(:,obj.num_q+1:end);
        
        constraint_force = -dpsidqd'*inv(dpsidqd*Hinv*dpsidqd')*(dpsidq*qd + dpsidqd*Hinv*(tau-C)+beta*psi);
      else
        constraint_force=0*q;
      end
      
%      [phi;psi]

      % todo: make this a more proper tolerance (which can be set
      % independently, or which is derived from the ode parameters?)
      % tol = 1e-4;
      % if (any(abs([phi;psi])>tol)) error('this state violates the bilateral constraints');  end
      
      qdd = Hinv*(tau + constraint_force - C);
      % note that I used to do this (instead of calling inv(H)):
      %   qdd = H\(tau - C)
      % but I already have and use Hinv, so use it again here
    end
    
    function [xdot, dxdot] = dynamics(obj,t,x,u)
      % Provides the dynamics interface for sodynamics.  This function
      % does not handle contact or joint limits!
      q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
      
      if nargout > 1
        if (obj.num_xcon>0)
          error('Not yet supported.');
        end
        
        % Note: the next line assumes that user gradients are implemented.
        % If it fails, then it will raise the same exception that I would
        % want to raise for this method, stating that not all outputs were
        % assigned.  (since I can't write dxdot anymore)
        [H,C,B,dH,dC,dB] = obj.manipulatorDynamics(q,qd);
        Hinv = inv(H);
        
        qdd = Hinv*(B*u - C);
        dxdot = [zeros(obj.num_q,1+obj.num_q), eye(obj.num_q),...
          zeros(obj.num_q,obj.num_u);...
          zeros(obj.num_q,1),...
          -Hinv*matGradMult(dH(:,1:obj.num_q),qdd) - Hinv*dC(:,1:obj.num_q),...
          -Hinv*dC(:,1+obj.num_q:end), Hinv*B];
        xdot = [qd;qdd];
      else
        qdd = obj.sodynamics(t,q,qd,u);
        xdot = [qd;qdd];
      end
    end    

%    function tau = computeConstraintTorques(q,qd,H,tau)
      % C should be C(q,q)qdot + G(q) - B*u
      
%    end
    
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
    
    function obj = setNumStateConstraints(obj,num)
      error('you must set position constraints and velocity constraints explicitly.  cannot set general constraints for manipulator plants');
    end
    
    function phi = positionConstraints(obj,q)
      error('manipulators with position constraints must overload this function');
    end
    
    function psi = velocityConstraints(obj,q,qd)
      error('manipulators with velocity constraints must overload this function'); 
    end
    
    function con = stateConstraints(obj,x)
      % wraps up the position and velocity constraints into the general constriant
      % method.  note that each position constraint (phi=0) also imposes an implicit
      % velocity constraint on the system (phidot=0).

      q=x(1:obj.num_q); qd=x(obj.num_q+1:end);
      if (obj.num_position_constraints>0)
        [phi,J] = geval(@obj.positionConstraints,q);
      else
        phi=[]; J=[];
      end
      if (obj.num_velocity_constraints>0)
        psi = obj.velocityConstraints(q,qd);
      else
        psi=[];
      end
      con = [phi; J*qd; psi];  % phi=0, phidot=0, psi=0
    end
    
    function sys = feedback(sys1,sys2)
      if (isa(sys2,'Manipulator'))
        % todo: implement this (or decide that it doesn't ever make sense)
        warning('feedback combinations of manipulators not handled explicitly yet. kicking out to a combination of SecondOrderSystems');
      end
      sys = feedback@SecondOrderSystem(sys1,sys2);
    end
    
    function sys = cascade(sys1,sys2)
      if (isa(sys2,'Manipulator'))
        % todo: implement this (or decide that it doesn't ever make sense)
        warning('cascade combinations of manipulators not handled explicitly yet. kicking out to a combination of SecondOrderSystems');
      end
      sys = cascade@SecondOrderSystem(sys1,sys2);
    end
    
    function polysys = makeTrigPolySystem(obj,options)
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
      
      polysys = makeTrigPolySystem@SecondOrderSystem(obj,options);
    end
    
  end
  
  
  properties (SetAccess = private, GetAccess = public)
    num_position_constraints = 0  % the number of position constraints of the form phi(q)=0
    num_velocity_constraints = 0  % the number of velocity constraints of the form psi(q,qd)=0
  end
end
