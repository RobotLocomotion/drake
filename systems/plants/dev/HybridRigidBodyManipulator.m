classdef HybridRigidBodyManipulator < HybridDrakeSystem

  methods
    function obj = HybridRigidBodyManipulator(model,options)
      if nargin<2, options=struct(); end
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        model = HybridRigidBodyMode(fullfile(pathname,filename),options);
      elseif ischar(model)
        model = HybridRigidBodyMode(model,options);
      elseif ~isa(model,'HybridRigidBodyMode')
        error('model must be a RigidBodyModel, PlanarRigidBodyModel or the name of a urdf file'); 
      end

      obj = obj@HybridDrakeSystem(getNumInputs(model),getNumOutputs(model));
      obj.model = model;  % note: this is an extra copy
      
      obj = setInputFrame(obj,getInputFrame(obj.model));
      obj = setOutputFrame(obj,getOutputFrame(obj.model));

      % just a single mode, with self-transitions
      obj = addMode(obj,model);
      
      constraint_ids = [obj.model.joint_limit_constraint_id,obj.model.position_constraint_ids];
      for i=1:numel(constraint_ids)
        lb = obj.model.state_constraints{constraint_ids(i)}.lb;
        ub = obj.model.state_constraints{constraint_ids(i)}.ub;
        for j=1:numel(lb)
          obj = addTransition(obj,1,@(obj,t,x,u)stateConstraintLimit(obj,t,x,u,constraint_ids(i),j,true),@(obj,from_mode_num,t,mode_x,u)activateStateConstraint(obj,from_mode_num,t,mode_x,u),true,true,1);
          obj = addTransition(obj,1,@(obj,t,x,u)stateConstraintLimit(obj,t,x,u,constraint_ids(i),j,false),@(obj,from_mode_num,t,mode_x,u)activateStateConstraint(obj,from_mode_num,t,mode_x,u),true,true,1);
        end
      end
    end
  
    function [phi,J] = stateConstraintLimit(obj,t,x,u,constraint_id,index,lb_or_ub)
      nx = obj.model.num_x;
      inds = false(nx);
      inds(obj.model.state_constraint_xind{constraint_id})=true;
      if nargout>1
        J = zeros(1,nx);
        [phi,Jc] = obj.model.state_constraints{constraint_id}.eval(x(inds));
        J(1,inds) = Jc(index,:);
      else
        phi = obj.model.state_constraints{constraint_id}.eval(x(inds));
      end
      phi=phi(index);
      if lb_or_ub
        phi = phi - obj.model.state_constraints{constraint_id}.lb(index);
      else
        phi = obj.model.state_constraints{constraint_id}.ub(index) - phi;
      end
    end
    
    function [xn,to_mode_num,status] = activateStateConstraint(obj,~,~,x,~)
      to_mode_num=1;
      status=0;
      
      q = x(1:obj.model.num_positions);
      v = x(obj.model.num_positions+1:end);

      H = manipulatorDynamics(obj.model,q,v);
      Hinv = inv(H);

      phi=[]; J=[]; dJ = []; lb=[]; ub=[];
      constraint_ids = [obj.model.joint_limit_constraint_id,obj.model.position_constraint_ids];
      
      for i=1:numel(constraint_ids)
        [this_phi,this_J] = obj.model.state_constraints{constraint_ids(i)}.eval(q);
        lb = [lb; obj.model.state_constraints{constraint_ids(i)}.lb];
        ub = [ub; obj.model.state_constraints{constraint_ids(i)}.ub];
        phi = [phi;this_phi];
        J = [J; this_J];
      end
      
      % find all active constraints (checking position only), and solve
      % for compatible velocities
      inds = phi<=lb | phi>=ub; % todo: add some tolerance here?
      J = J(inds,:)*vToqdot(obj.model, q);
      
      vn = (eye(obj.model.num_velocities)-Hinv*J'*pinv(J*Hinv*J')*J)*v;
      xn = [q;vn];
    end    
    
    function v=constructVisualizer(obj,varargin)
      v = constructVisualizer(obj.model,varargin{:});
    end
  end

  
  properties
    model
  end
end
