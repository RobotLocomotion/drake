classdef RigidBodyBuoyant < RigidBodyForceElement

  properties
    kinframe;  % index to RigidBodyFrame 
    volume; %volume of the body
    rho; % density of fluid
  end
  
  methods
    function obj = RigidBodyBuoyant(frame_id, rho, volume)
      % creates a force element that adds buoyancy at new frame location
      % @param frame_id = RigidBodyFrame specifying the location of center
      % of volume
      % @param rho = density of surrounding fluid
      % @param volume = volume of the body
      
      typecheck(frame_id,'numeric');
      obj.kinframe = frame_id;
      obj.rho = rho;
      obj.volume = volume;
      
    end 
    function force = computeSpatialForce(obj,manip,q,qd)
      frame = getFrame(manip,obj.kinframe);
      kinsol = doKinematics(manip,q);

      force = sparse(6,getNumBodies(manip));
      buoyancy_world = -manip.gravity*obj.rho*obj.volume;
      force(:,frame.body_ind) = cartesianForceToSpatialForce(manip, kinsol, frame.body_ind, frame.T(1:3,4),buoyancy_world);
    end
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
      elNode = node.getElementsByTagName('parent').item(0);
      parent = findLinkId(model,char(elNode.getAttribute('link')),robotnum);
      
      xyz=zeros(3,1); rpy=zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
        if elnode.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
        end
      end
      [model,frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));
      
      rho = parseParamString(model,robotnum,char(node.getAttribute('rho')));
      volume = parseParamString(model,robotnum,char(node.getAttribute('volume')));
      
      obj = RigidBodyBuoyant(frame_id, rho, volume);
      
      obj.name = name;
    end
  end
  
end
