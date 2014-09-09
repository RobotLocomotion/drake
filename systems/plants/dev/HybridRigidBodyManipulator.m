classdef HybridRigidBodyManipulator < HybridDrakeSystem
%
% note: this model currently assumes frictionless impacts 
% see Posa13 for more details / options.
% (todo:  add warning on this?)



  methods
    function obj = HybridRigidBodyManipulator(model)
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        model = RigidBodyModel(fullfile(pathname,filename));
      elseif ischar(model)
        model = RigidBodyModel(model);
      elseif ~isa(model,'RigidBodyModel')
        error('model must be a RigidBodyModel, PlanarRigidBodyModel or the name of a urdf file'); 
      end

      obj = obj@HybridDrakeSystem(size(model.B,2),2*model.featherstone.NB);
      obj.model = model;
      
      % now construct all of the modes
      % each joint has 1,2, or 3 permutations, depending if there
      % are no limits, limits on one side, or limits on both sides.
      %  (should I really worry about the one side case?)
      % each contact has 3 permutations - no contact, non-sliding contact,
      % and sliding contact.
      
      obj.joint_limit_min = [obj.model.body.joint_limit_min]';
      obj.joint_limit_max = [obj.model.body.joint_limit_max]';
      obj.num_contacts = size([obj.model.body.contact_pts],2);
      
      nL = sum([obj.joint_limit_min~=-inf | obj.joint_limit_max~=inf]); 
      nC = obj.num_contacts;
      N = 3^(nC+nL);
      
      in_frame = constructInputFrame(obj.model);
      state_frame = constructStateFrame(obj.model);
      out_frame = state_frame;
      
      obj = setInputFrame(obj,in_frame);
      obj = setOutputFrame(obj,out_frame);
      
      % create modes
      for i=1:N
        [joint_limit_state,contact_state] = modeNumToConstraints(obj,i);
        name = ['joint_limit_state = ',mat2str(joint_limit_state'),', contact_state = ',mat2str(contact_state)];
        obj = addMode(obj,HybridRigidBodyMode(model,joint_limit_state,contact_state,in_frame,state_frame,out_frame),name);
      end
      
      % create the transitions
      for i=1:N
        %    phi = guard(fsm_obj,t,mode_x,u)
        %    [to_mode_xn,to_mode_num,status] = transition(obj,from_mode_num,t,mode_x,u)
      end
      
    end
  
    function modeNum = constraintsToModeNum(obj, joint_limit_state, contact_state)
      % @param joint_limit_state a vector the size of num_q with
      % each value being 0, 1, 0 for nolim, min, or max.
      % @param contact_state a vector of size obj.manip.num_contacts that is
      % 0 for no contact, 1 for non-sliding contact, and 2 for sliding
      % contact
      % @retval modeNum the integer number for the corresponding mode
      typecheck(joint_limit_state,'double');
      typecheck(contact_state,'double');
      
      joint_w_limits = [obj.manip.joint_limit_min~=-inf | obj.manip.joint_limit_max~=inf];
      nL = sum(joint_w_limits);
      nC = obj.num_contacts;
      
      sizecheck(joint_limit_state,[obj.manip.num_positions,1]);
      sizecheck(contact_state,[nC,1]);
      
      if (~all(joint_limit_state==0 | joint_limit_state ==1 | joint_limit_state==2))
        error('joint limit state must be one of {0,1,2}');
      end
      if (~all(contact_state==0 | contact_state==1 | contact_state==2))
        error('contact state must be one of {0,1,2}');
      end
      
      modeNum = 1+sum((3.^((nL+nC-1):-1:0)'.*[joint_limit_state;contact_state]));
    end
    
    function [joint_limit_state,contact_state] = modeNumToConstraints(obj, modeNum)
      % inverse of constraintToModeNum
      joint_w_limits = [obj.joint_limit_min~=-inf | obj.joint_limit_max~=inf];
      nL = sum(joint_w_limits);
      nC = obj.num_contacts;
      
      sizecheck(modeNum,1);
      if modeNum<1 || modeNum> 3^(nC+nL)
        error('modeNum is out of range');
      end
      
      state=rem(floor((modeNum-1)*3.^(1-(nL+nC):0)),3);
      joint_limit_state = zeros(length(obj.joint_limit_min),1);
      joint_limit_state(joint_w_limits) = state(1:nL);
      contact_state = state(nL+1:end);
    end

    function [phi,J,dJ] = jointLimitActiveGuard(obj,q)
      phi = [q-obj.joint_limit_min; obj.joint_limit_max-q]; phi=phi(~isinf(phi));
      J = [eye(obj.num_positions); -eye(obj.num_positions)];  
      J([obj.joint_limit_min==-inf;obj.joint_limit_max==inf],:)=[]; 
      if (nargout>2)
        dJ = sparse(length(phi),obj.num_positions^2);
      end
    end
    
    function v=constructVisualizer(obj)
      options=struct();
      if (obj.num_contacts)
        options.ground=true;
      end
      if isa(obj.model,'PlanarRigidBodyModel')
        v = PlanarRigidBodyVisualizer(obj.getOutputFrame,obj.model,options);
      else
        v = RigidBodyWRLVisualizer(obj.getOutputFrame,obj.model,options);
      end
    end
  end

  
  properties
    model
    joint_limit_min
    joint_limit_max
    num_contacts
  end
end
