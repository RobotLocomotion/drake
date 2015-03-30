classdef RigidBodyThrust < RigidBodyForceElement

  properties
    kinframe
    axis
    scale_factor = 1; %amount to scale the input by.
  end
  
  methods
    function obj = RigidBodyThrust(frame_id, axis, scale_factor, limits)
      
      obj.kinframe = frame_id;
      obj.axis = axis/norm(axis);      
      
      obj.direct_feedthrough_flag = true;
      obj.input_num = 0;
      obj.input_limits = [-inf inf];
      if (nargin > 2)
        obj.scale_factor = scale_factor;
      end
      if (nargin > 3)
        obj.input_limits = limits;
      end
    end %constructor
    
    function [force, B_mod, dforce, dB_mod] = computeSpatialForce(obj,manip,q,qd)
      %B_mod maps the input to generalized forces.
      
      force = sparse(6,getNumBodies(manip))*q(1);
      B_mod = manip.B*0*q(1); %initialize B_mod

      if (nargout>2)  % then compute gradients
        kinsol = doKinematics(manip,q,true);
        [x,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
        [axis_world,Jaxis_world] = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        daxis_world = Jaxis_world-J;
        axis_world = axis_world-x;

        dforce = sparse(6*getNumBodies(manip),getNumStates(manip));

        nq = getNumPositions(manip); nu = getNumInputs(manip);
        dB_mod = sparse(nq*nu,getNumStates(manip));
        dB_mod((obj.input_num-1)*nq + (1:nq),1:nq) = obj.scale_factor*(J'*daxis_world + reshape(dJ'*axis_world,nq,nq));
      else
        kinsol = doKinematics(manip,q);
        [x,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
        axis_world = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        axis_world = axis_world-x;
      end
      
      % apply force along the z-axis of the reference frame
      B_mod(:,obj.input_num) = obj.scale_factor*J'*axis_world;
    end
    
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
      elnode = node.getElementsByTagName('parent').item(0);
      parent = findLinkId(model,char(elnode.getAttribute('link')),robotnum);
      
      xyz = zeros(3,1); rpy = zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
        if elnode.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
        end
      end
      fr = RigidBodyFrame(parent,xyz,rpy,[name,'_frame']);
      [model,frame_id] = addFrame(model,fr);
      
      scaleFac = 1;
      if node.hasAttribute('scale_factor')
        scaleFac = parseParamString(model,robotnum,char(node.getAttribute('scale_factor')));
      end
      
      axis = [1; 0; 0];
      elnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(elnode)
        axis = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        axis = axis/(norm(axis)+eps); % normalize
      end
      
      limits = [-inf,inf];
      if node.hasAttribute('lower_limit')
        limits(1) = parseParamString(model,robotnum,char(node.getAttribute('lower_limit')));
      end
      if node.hasAttribute('upper_limit')
        limits(2) = parseParamString(model,robotnum,char(node.getAttribute('upper_limit')));
      end
      
      obj = RigidBodyThrust(frame_id, axis, scaleFac, limits);
      obj.name = name;
    end
  end
  
end
