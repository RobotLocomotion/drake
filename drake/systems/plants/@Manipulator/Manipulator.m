classdef Manipulator < DrakeSystem
% An abstract class that wraps H(q)vdot + C(q,v,f_ext) = B(q)u.

  methods
    function obj = Manipulator(num_q, num_u, num_v)
      if nargin<3, num_v = num_q; end

      obj = obj@DrakeSystem(num_q+num_v,0,num_u,num_q+num_v,false,true);
      obj.num_positions = num_q;
      obj.num_velocities = num_v;
      obj.joint_limit_min = -inf(num_q,1);
      obj.joint_limit_max = inf(num_q,1);
    end
  end

  methods (Abstract=true)
    %  H(q)vdot + C(q,v,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,v);
  end

  methods
    function [H,C_times_v,G,B] = manipulatorEquations(obj,q,v)
      % extract the alternative form of the manipulator equations:
      %   H(q)vdot + C(q,v)v + G(q) = B(q)u
      
      if nargin<2,
        checkDependency('spotless');
        q = TrigPoly('q','s','c',getNumPositions(obj));
      end
      if nargin<3,
        v = msspoly('v',getNumVelocities(obj));
      end
      
      [H,C,B] = manipulatorDynamics(obj,q,v);
      [~,G] = manipulatorDynamics(obj,q,0*v);
      C_times_v = C-G;
    end
    
    function [xdot,dxdot] = dynamics(obj,t,x,u)
    % Provides the DrakeSystem interface to the manipulatorDynamics.

      q = x(1:obj.num_positions);
      v = x(obj.num_positions+1:end);

      if (nargout>1)
        if ~isempty(obj.position_constraints) || ~isempty(obj.velocity_constraints)
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
          dtau = matGradMult(dB,u) - dC;
          dvdot = [zeros(obj.num_velocities,1),...
            -Hinv*matGradMult(dH(:,1:obj.num_positions),vdot) + Hinv*dtau(:,1:obj.num_positions),...
            +Hinv*dtau(:,1+obj.num_positions:end), Hinv*B];
        else
          vdot = -Hinv*C;
          dvdot = [zeros(obj.num_velocities,1),...
            Hinv*(-matGradMult(dH(:,1:obj.num_positions),vdot) - dC(:,1:obj.num_positions)),...
            Hinv*(-dC(:,obj.num_positions+1:end))];
        end

        kinsol = obj.doKinematics(q, [], struct('compute_gradients', true));
        [VqInv,dVqInv] = vToqdot(obj,kinsol);
        xdot = [VqInv*v;vdot];
        dxdot = [...
          zeros(obj.num_positions,1), matGradMult(dVqInv, v), VqInv, zeros(obj.num_positions,obj.num_u);
          dvdot];
      else
        [H,C,B] = manipulatorDynamics(obj,q,v);
        Hinv = inv(H);
        if (obj.num_u>0) tau=B*u - C; else tau=-C; end
        tau = tau + computeConstraintForce(obj,q,v,H,tau,Hinv);

        vdot = Hinv*tau;
        % note that I used to do this (instead of calling inv(H)):
        %   vdot = H\tau
        % but I already have and use Hinv, so use it again here

        kinsol = obj.doKinematics(q);
        xdot = [vToqdot(obj,kinsol)*v; vdot];
      end

    end

    function [Vq,dVq] = qdotToV(obj, kinsol)
      % defines the linear map v = Vq * qdot
      % default relationship is that v = qdot
      assert(obj.num_positions==obj.num_velocities);
      Vq = eye(length(kinsol.q));
      dVq = zeros(numel(Vq), obj.num_positions);
    end

    function [VqInv,dVqInv] = vToqdot(obj, kinsol)
      % defines the linear map qdot = Vqinv * v
      % default relationship is that v = qdot
      assert(obj.num_positions==obj.num_velocities);
      VqInv = eye(length(kinsol.q));
      dVqInv = zeros(numel(VqInv), obj.num_positions);
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

      alpha = 5;  % 1/time constant of position constraint satisfaction (see my latex rigid body notes)
      beta = 0;    % 1/time constant of velocity constraint satisfaction

      phi=[]; psi=[];
      kinsol = obj.doKinematics(q);
      qd = vToqdot(obj, kinsol) * v;
      if ~isempty(obj.position_constraints) && ~isempty(obj.velocity_constraints)
        [phi,J,dJ] = obj.positionConstraints(q);
        Jdotqd = dJ*reshape(qd*qd',obj.num_positions^2,1);

        [psi,dpsi] = obj.velocityConstraints(q,qd);
        dpsidq = dpsi(:,1:obj.num_positions);
        dpsidqd = dpsi(:,obj.num_positions+1:end);

        term1=Hinv*[J;dpsidqd]';
        term2=Hinv*tau;

        constraint_force = -[J;dpsidqd]'*pinv([J*term1;dpsidqd*term1])*[J*term2 + Jdotqd + 2*alpha*J*qd + alpha^2*phi; dpsidqd*term2 + dpsidq*qd + beta*psi];
      elseif ~isempty(obj.position_constraints)  % note: it didn't work to just have dpsidq,etc=[], so it seems like the best solution is to handle each case...
        [phi,J,dJ] = obj.positionConstraints(q);
        % todo: find a way to use Jdot*qd directly (ala Twan's code)
        % instead of computing dJ
        Jdotqd = dJ*reshape(qd*qd',obj.num_positions^2,1);

        constraint_force = -J'*pinv(J*Hinv*J')*(J*Hinv*tau + Jdotqd + 2*alpha*J*qd + alpha^2*phi);
        % useful for debugging:
        % phi, phidot = J*qd, phiddot = J*Hinv*(tau+constraint_force)+Jdotqd
      elseif ~isempty(obj.velocity_constraints)
        [psi,J] = obj.velocityConstraints(q,qd);
        dpsidq = J(:,1:obj.num_positions);
        dpsidqd = J(:,obj.num_positions+1:end);

        constraint_force = -dpsidqd'*pinv(dpsidqd*Hinv*dpsidqd')*(dpsidq*qd + dpsidqd*Hinv*tau+beta*psi);
      else
        constraint_force = 0*v;
      end
    end
  end

  methods
    function [obj,id] = addPositionEqualityConstraint(obj,con,position_ind)
      % Adds a position constraint of the form phi(q) = constant
      % which can be enforced directly in the manipulator dynamics.
      % This method will also register phi (and it's time derivative)
      % as state constraints for the dynamical system.
      if( isa(con, 'DrakeFunctionConstraint') && ~isa(con.fcn, 'drakeFunction.kinematic.RelativePosition'))
        obj.only_loops = false;
      end
      if (nargin<3 || isempty(position_ind)) 
        position_ind = 1:getNumPositions(obj);
      end
      id = numel(obj.position_constraints)+1;
      obj = updatePositionEqualityConstraint(obj,id,con,position_ind);
    end
    
    function obj = updatePositionEqualityConstraint(obj,id,con,position_ind)
      typecheck(con,'Constraint'); % for now
      assert(all(con.lb == con.ub));
      if (nargin<4 || isempty(position_ind)) 
        position_ind = 1:getNumPositions(obj);
      end
        
      if id>numel(obj.position_constraints) % then it's a new constraint
        [obj,obj.position_constraint_ids(1,id)] = addStateConstraint(obj,con,position_ind);
      else 
        obj.num_position_constraints = obj.num_position_constraints-obj.position_constraints{id}.num_cnstr;
        obj = updateStateConstraint(obj,obj.position_constraint_ids(1,id),con,position_ind);
      end
      
      obj.num_position_constraints = obj.num_position_constraints+con.num_cnstr;
      obj.position_constraints{id} = con;
      
%      obj.warning_manager.warnOnce('Drake:Manipulator:Todo','still need to add time derivatives of position constraints');
    end
    
    function [obj,id] = addVelocityEqualityConstraint(obj,con,velocity_ind)
      % Adds a velocity constraint of the form psi(q,v) = constant
      % (with dpsidv~=0) which can be enforced directly in the manipulator dynamics.
      % This method will also register psi as a state constraint
      % for the dynamical system.
      if (nargin<3 || isempty(velocity_ind))
        velocity_ind = 1:getNumVelocities(obj);
      end
      
      id = numel(obj.position_constraints)+1;
      obj = updatePositionEqualityConstraint(obj,id,con);
      
    end
    function obj = updateVelocityEqualityConstraint(obj,id,con,velocity_ind)
      typecheck(con,'Constraint'); % for now
      assert(all(con.lb == con.ub));
      if (nargin<4 || isempty(velocity_ind))
        velocity_ind = 1:getNumVelocities(obj);
      end
        
      if id>numel(obj.velocity_constraints) % then it's a new constraint
        [obj,obj.velocity_constraint_ids(1,id)] = addStateConstraint(obj,con,getNumPositions(obj) + velocity_ind);
      else 
        obj.num_velocity_constraints = obj.num_velocity_constraints-obj.velocity_constraints{id}.num_cnstr;
        obj = updateStateConstraint(obj,obj.velocity_constraint_ids(1,id),con,getNumPositions(obj) + velocity_ind);
      end
      
      obj.num_velocity_constraints = obj.num_velocity_constraints+con.num_cnstr;
      obj.velocity_constraints{id} = con;
    end
    
    function obj = removeAllStateConstraints(obj)
      obj.num_position_constraints=0;
      obj.num_velocity_constraints=0;
      obj.position_constraints = {};  
      obj.position_constraint_ids = [];
      obj.velocity_constraints = {};
      obj.velocity_constraint_ids = []; 
      obj.joint_limit_constraint_id = [];
      obj = removeAllStateConstraints@DrakeSystem(obj);
    end
  end

  methods
    function obj = setNumDOF(obj,num)
      warnOnce(obj.warning_manager,'Drake:Manipulator:setNumDOFDeprecated','setNumDOF will soon be deprecated.  In order to fully support quaternion floating base dynamics, we had to change the interface to allow a different number position and velocity elements in the state vector.  Use setNumPositions() and setNumVelocities() instead.');
      obj.setNumPositions(num);
      obj.setNumVelocities(num);
%       error('setNumDOF is deprecated.  Use setNumPositions and setNumVelocities instead.');
    end
    function n = getNumDOF(obj)
      warnOnce(obj.warning_manager,'Drake:Manipulator:getNumDOFDeprecated','getNumDOF will soon be deprecated.  In order to fully support quaternion floating base dynamics, we had to change the interface to allow a different number position and velocity elements in the state vector.  Use getNumPositions() and getNumVelocities() instead.');
      n = obj.getNumPositions();
%       error('getNumDOF is deprecated.  In order to fully support quaternion floating base dynamics, we had to change the interface to allow a different number position and velocity elements in the state vector.  Use getNumPositions() and getNumVelocities() instead.');
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
    
    function varargout = positionConstraints(obj,q)
      % Implements position constraints of the form phi(q) = 0
      
      varargout = cell(1,nargout);
      for i=1:length(obj.position_constraints)
        v = cell(1,max(nargout,1));
        [v{:}] = obj.position_constraints{i}.eval(q);
        v{1} = v{1} - obj.position_constraints{i}.lb;  % center it around 0
        for j=1:nargout
          varargout{j} = vertcat(varargout{j},v{j});
        end
      end
    end

    function varargout = velocityConstraints(obj,q,v)
      % Implements velocity constraints of the form psi(q,v) = 0
      % Note: dphidv must not be zero. constraints which depend
      % only on q should be implemented instead as positionConstraints.

      varargout = cell(1,nargout);
      for i=1:length(obj.velocity_constraints)
        v = cell(1,nargout);
        [v{:}] = obj.velocity_constraints{i}.eval(x);
        v{1} = v{1} - obj.velocity_constraints{i}.lb;  % center it around 0
        for j=1:nargout
          varargout{j} = vertcat(varargout{j},v{j});
        end
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

    function sys = feedback(sys1,sys2)
      % Attempt to produce a new manipulator system if possible

      if (isa(sys2,'Manipulator'))
        % todo: implement this (or decide that it doesn't ever make sense)
        warning('feedback combinations of manipulators not handled explicitly yet. kicking out to a combination of DrakeSystems');
      end
      sys = feedback@DrakeSystem(sys1,sys2);
    end

    function sys = cascade(sys1,sys2)
      % Attempt to produce a new manipulator system if possible

      if (isa(sys2,'Manipulator'))
        % todo: implement this (or decide that it doesn't ever make sense)
        warning('cascade combinations of manipulators not handled explicitly yet. kicking out to a combination of DrakeSystems');
      end
      sys = cascade@DrakeSystem(sys1,sys2);
    end

    function polysys = extractTrigPolySystem(obj,options)
      % Creates a (rational) polynomial system representation of the
      % dynamics

      if getNumStateConstraints(obj)>0 || getNumUnilateralConstraints(obj)>0, error('not implemented yet.  may not be possible.'); end

      function rhs = dynamics_rhs(obj,t,x,u)
        q=x(1:obj.num_positions); v=x((obj.num_positions+1):end);
        [~,C,B] = manipulatorDynamics(obj,q,v);
        if (obj.num_u>0) tau=B*u; else tau=zeros(obj.num_u,1); end
        kinsol = obj.doKinematics(q);
        rhs = [vToqdot(obj,kinsol)*v;tau - C];
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

      % todo: consider adding an option that would give [q_d;qd_d] as in
      % input.  would be trivial to type in, but doesn't add any richness
      % to the control input (and increases the dimensionality)

      % todo: consider allowing the user to specify less than num_u gains
      % (e.g., PD control just part of the system).  And pass the original
      % input through on the other inputs.

      if nargin<4 || isempty(index)
        % try to extract the index from B
        checkDependency('spotless');
        q=msspoly('q',sys.num_positions);
        s=msspoly('s',sys.num_positions);
        c=msspoly('c',sys.num_positions);
        qt=TrigPoly(q,s,c);
        qd=msspoly('v',sys.num_positions);

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
          warning('Drake:Manipulator:PDControlDefaultIndex','Couldn''t extract default index from the B matrix.  resorting to default behavior.');
          warning(lasterr);
          index=[];
        end
      end

      varargout=cell(1,nargout);

      sizecheck(Kp,[sys.num_u,sys.num_u]);
      sizecheck(Kd,[sys.num_u,sys.num_u]);

      if nargin<4 || isempty(index)
        index = 1:sys.num_u;
      end
      sizecheck(index,sys.num_u);
      rangecheck(index,0,sys.num_positions);

      % pdfb = prop-derivative control feedback term:
      % tau = -Kp*theta - Kd*thetadot
      D = zeros(sys.num_u,sys.num_x);
      D(:,index) = -Kp;
      D(:,sys.num_positions + index) = -Kd;
      pdfb = LinearSystem([],[],[],[],[],D);
      pdfb = setOutputFrame(pdfb,sys.getInputFrame);
      pdfb = setInputFrame(pdfb,sys.getStateFrame);  % note: assume full-state feedback for now

      % pdff = prop-derivative control feedforward term:
      % tau = Kp*thetadesired
      pdff = LinearSystem([],[],[],[],[],Kp*eye(sys.num_u));
      pdff = setOutputFrame(pdff,sys.getInputFrame);
      coordinates = sys.getStateFrame.getCoordinateNames();
      pdff = setInputFrame(pdff,CoordinateFrame('q_d',length(index),'d',{coordinates{index}}));

      if nargout>1
        varargout{1} = pdff;
        varargout{2} = pdfb;
      else
%        sys = cascade(cascade(ScopeSystem(sys.getInputFrame),sys),ScopeSystem(sys.getInputFrame));
        varargout{1} = cascade(pdff,feedback(sys,pdfb));
      end
    end

    function [jl_min, jl_max] = getJointLimits(obj)
      % Returns lower and upper joint limit vectors
      jl_min = obj.joint_limit_min;
      jl_max = obj.joint_limit_max;
    end

    function obj = setJointLimits(obj,jl_min,jl_max)
      sizecheck(jl_min,[obj.num_positions,1]);
      sizecheck(jl_max,[obj.num_positions,1]);
      assert(all(jl_max>=jl_min));
      obj.joint_limit_min = jl_min;
      obj.joint_limit_max = jl_max;
      
      con = BoundingBoxConstraint(jl_min,jl_max);
      con = setName(con,cellfun(@(a) [a,'_limit'],obj.getPositionFrame.getCoordinateNames(),'UniformOutput',false));
      if isempty(obj.joint_limit_constraint_id)
        if any(jl_min~=-inf) || any(jl_max~=inf)
          [obj,id] = addStateConstraint(obj,con,1:obj.getNumPositions);
          obj.joint_limit_constraint_id = id;
        end
      else
        % todo: delete the constraint if [-inf,inf] (but don't have a good
        % mechanism for removing state constraints yet)
        obj = updateStateConstraint(obj,obj.joint_limit_constraint_id,con,1:obj.getNumPositions);
      end
    end
    
  end  

  properties (SetAccess = private, GetAccess = public)
    num_positions=0;
    num_velocities=0;
    num_position_constraints=0;
    num_velocity_constraints=0;
    position_constraints = {};  % position equality constraints of the form phi(q)=const
    position_constraint_ids = []; % also have to keep associate state constraints
    velocity_constraints = {};  % velocity equality constraints of the form psi(q,qd)=const
    velocity_constraint_ids = []; % also keep associated state constraints
    joint_limit_min = -inf;       % vector of length num_q with lower limits
    joint_limit_max = inf;        % vector of length num_q with upper limits
    joint_limit_constraint_id = [];
    only_loops = true; %set to false when the manipulator gets non-loop constraints
  end
end
