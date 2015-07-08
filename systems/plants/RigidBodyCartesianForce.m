classdef RigidBodyCartesianForce < RigidBodyForceElement

  properties
    kinframe
  end
  
  methods
    function obj = RigidBodyCartesianForce(name,frame_id,limits)
      obj.name = name;
      obj.kinframe = frame_id;
      obj.direct_feedthrough_flag = true;
      obj.input_limits = repmat([-inf inf],3,1);
      if (nargin > 2)
        sizecheck(limits,size(obj.input_limits));
        obj.input_limits = limits;
      end
    end %constructor
    
    function [force, B_mod, dforce, dB_mod] = computeSpatialForce(obj,manip,q,qd)
      %B_mod maps the input to generalized forces.
      
      force = sparse(6,getNumBodies(manip))*q(1);
      B_mod = manip.B*0*q(1); %initialize B_mod

      if (nargout>2)  % then compute gradients
        kinsol = doKinematics(manip,q,true);
        [~,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));

        dforce = sparse(6*getNumBodies(manip),getNumStates(manip));

        nq = getNumPositions(manip); nu = getNumInputs(manip);
        assert(nq == getNumVelocities(manip));
        dB_mod = sparse(nq*nu,getNumStates(manip));
        dB_mod((obj.input_num-1)*nq + (1:3*nq),1:nq) = reshape(dJ',3*nq,nq);
      else
        kinsol = doKinematics(manip,q);
        [~,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
      end
      
      B_mod(:,obj.input_num) = J';
    end
    
    function fr = constructFrame(obj,manip)
      fr = CoordinateFrame(obj.name,3,'f',{'x','y','z'});
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
      
      obj = RigidBodyCartesianForce(name,frame_id);
    end
  end
  
end
