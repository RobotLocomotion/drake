classdef HybridRigidBodyManipulator < HybridDrakeSystem

  methods
    function obj = HybridRigidBodyManipulator(mode_sys,options)
      if nargin<2, options=struct(); end
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        mode_sys = HybridRigidBodyMode(fullfile(pathname,filename),options);
      elseif ischar(mode_sys)
        mode_sys = HybridRigidBodyMode(mode_sys,options);
      elseif ~isa(mode_sys,'HybridRigidBodyMode')
        error('model must be a RigidBodyManipulator or the name of a urdf file'); 
      end

      obj = obj@HybridDrakeSystem(getNumInputs(mode_sys),getNumOutputs(mode_sys));
      obj.model = mode_sys.model;  % note: this is an extra copy
      
      obj = setInputFrame(obj,getInputFrame(obj.model));
      obj = setOutputFrame(obj,getOutputFrame(obj.model));

      % just a single mode, with self-transitions
      obj = addMode(obj,mode_sys);
      
      constraint_ids = [obj.model.joint_limit_constraint_id,obj.model.position_constraint_ids];
      constraint_index = 0;
      for id=constraint_ids
        lb = obj.model.state_constraints{id}.lb;
        ub = obj.model.state_constraints{id}.ub;
        for j=1:numel(lb)
          constraint_index=constraint_index+1;
          if (lb(j)==ub(j)), continue; end % this one will never cause a transition
          if ~isinf(lb(j))
            % add lb transition
            obj = addTransition(obj,1,@(obj,t,x,u)stateConstraintLimit(obj,t,x,u,id,j,true,constraint_index),@(obj,from_mode_num,t,mode_x,u)flipStateConstraint(obj,from_mode_num,t,mode_x,u,true,constraint_index),false,true,1);
          end
          if ~isinf(ub(j))
            % add ub transition
            obj = addTransition(obj,1,@(obj,t,x,u)stateConstraintLimit(obj,t,x,u,id,j,false,constraint_index),@(obj,from_mode_num,t,mode_x,u)flipStateConstraint(obj,from_mode_num,t,mode_x,u,false,constraint_index),false,true,1);
          end
        end
      end
    end
      
    function [phi,J] = stateConstraintLimit(obj,~,x,~,constraint_id,index,lb_not_ub,constraint_state_index)
      % this method implements a number of different guards to handle all
      % of the position constraints:
      %  if the constraint is not currently active, then it implements the
      %  activation guards
      %     phi<=lb OR phi>=ub  % note: it currently does this as two
      %                           separate guards but could do it as one OR guard
      %  if the constraint is not currently active, then it implements the 
      %  deactivation guards
      %     phi>=lb+tolerance
      %       OR
      %     phi<=ub-tolerance
      
      nxc = obj.model.num_xc;
      nu = obj.model.num_u;
      
      num_constraints = obj.modes{1}.num_xd;
      constraint_state = x(1:num_constraints);
      x = x((num_constraints+1):end);
      
      constraint_already_active = (lb_not_ub && (constraint_state(constraint_state_index)<-.5 || constraint_state(constraint_state_index)>1.5)) || ...
        (~lb_not_ub && constraint_state(constraint_state_index)>.5);
      
      inds = false(1,nxc);
      inds(obj.model.state_constraint_xind{constraint_id})=true;
      if nargout>1 || constraint_already_active;
        J = zeros(1,1+nxc+nu);
        [phi,Jc] = obj.model.state_constraints{constraint_id}.eval(x(inds));
        J(1,[false,inds,false(1,nu)]) = Jc(index,:);
      else
        J=[];
        phi = obj.model.state_constraints{constraint_id}.eval(x(inds));
      end
      phi=phi(index);
      if lb_not_ub
        phi = phi - obj.model.state_constraints{constraint_id}.lb(index);
      else
        phi = obj.model.state_constraints{constraint_id}.ub(index) - phi;
        J = -J;
      end
      
      if constraint_already_active
        % then this constraint is already active, so flip the sign to use
        % this as a de-activation guard
        phi = -phi+1e-4; % add a little tolerance here
        J = -J;
      end
    end
    
    function [xn,to_mode_num,status] = flipStateConstraint(obj,~,~,x,~,lb_not_ub,constraint_state_index)
      to_mode_num=1;
      status=0;

      num_constraints = obj.modes{1}.num_xd;
      constraint_state = x(1:num_constraints);
      x = x((num_constraints+1):end);
      
      % update the constraint state
      activating_constraint = false;  % true if a new constraint is being turned ON
      state = constraint_state(constraint_state_index);
      assert(state<1.5); % shouldn't ever get here for a constraint that is active in both LB and UB (because that state will never transition)
      if (lb_not_ub)  % flipping LB constraint
        if (state<-0.5) % only LB was on
          state = 0;
        else % nothing was on
          activating_constraint = true;
          state = -1;
        end
      else
        if (state>0.5)
          state = 0;
        else
          activating_constraint = true;
          state = 1;
        end
      end
      constraint_state(constraint_state_index) = state;
      
      if (activating_constraint)
        q = x(1:obj.model.num_positions);
        v = x(obj.model.num_positions+1:end);
        
        H = manipulatorDynamics(obj.model,q,v);
        Hinv = inv(H);
        
        phi=[]; J=[]; dJ = []; lb=[]; ub=[];
        
        for id=obj.modes{1}.constraint_ids
          [this_phi,this_J] = obj.model.state_constraints{id}.eval(q);
          lb = [lb; obj.model.state_constraints{id}.lb];
          ub = [ub; obj.model.state_constraints{id}.ub];
          phi = [phi;this_phi];
          J = [J; this_J];
        end
        
        % solve zero post-transition velocities for all active constraints
        % was: inds = phi<=lb | phi>=ub; % todo: add some tolerance here?
        inds = abs(constraint_state)>0.5;
        
        J = J(inds,:)*vToqdot(obj.model, q);
        
        vn = (eye(obj.model.num_velocities)-Hinv*J'*pinv(J*Hinv*J')*J)*v;
        xn = [constraint_state;q;vn];
      else
        xn = [constraint_state;x];
      end
      
    end    
    
    function v=constructVisualizer(obj,varargin)
      v = constructVisualizer(obj.model,varargin{:});
    end
  end

  
  properties
    model
  end
end
