classdef RigidBodyThrust < RigidBodyForceElement

  properties
    bodyind    % RigidBody itndex of the parent
    origin=zeros(3,1);  %xyz of the reference point for the thrust element
    direction = [1 0 0]' %initially along the x-axis of the parent
    scaleFactor = 1; %amount to scale the input by.
  end
  
  methods
    function obj = RigidBodyThrust(parent_body, orig, dir, scaleFac, limits)
        if ~isa(parent_body, 'numeric');
            error('Drake:RigidBodyThrust:InvalidParent','Force Type Thrust does not have a proper RigidBody parent');
        end
        obj.bodyind = parent_body;
        obj.origin = orig;
        %Normalize direction vector, in case user didn't
        dir = dir/norm(dir);
        obj.direction = reshape(dir,3,1);
        obj.direct_feedthrough_flag = true;
        obj.input_num = 0;
        obj.input_limits = [-inf inf];
        if (nargin > 3)
            obj.scaleFactor = scaleFac;
        end
        if (nargin > 4)
            obj.input_limits = limits;
        end
    end %constructor
    
    function [force, B_mod] = computeSpatialForce(obj,manip,q,qd)
        %B_mod maps the input to generalized forces.
        kinsol = doKinematics(manip,q, false, false);
        %origin = [x y z] of the reference point
        [x,J] = forwardKin(manip,kinsol,obj.bodyind,obj.origin(1:3)', 0);
        B_mod = manip.B*0; %initialize B_mod
        force = sparse(6,getNumBodies(manip));
        %J expects the force to be in world coordinates
        kinsolT = kinsol.T{obj.bodyind};
        direction_world = kinsolT(1:3,1:3)*obj.direction;
        %B_mod(:,obj.input_num) = J'*obj.direction*obj.scaleFactor;
        B_mod(:,obj.input_num) = J'*direction_world*obj.scaleFactor;
        B_mod;
    end
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.bodyind = map_from_old_to_new(obj.bodyind);
    end
    
  end
  
  methods (Static)
    function obj = parseURDFNode(model,robotnum,node,options)
      elNode = node.getElementsByTagName('parent').item(0);
      parent = findLinkInd(model,char(elNode.getAttribute('link')),robotnum);
      
      elnode = node.getElementsByTagName('origin').item(0);
      orig = parseParamString(model,robotnum,char(elnode.getAttribute('xyz')));
      
      elnode = node.getElementsByTagName('direction').item(0);
      dir = parseParamString(model,robotnum,char(elnode.getAttribute('xyz')));
      
      scaleFac = 1;
      elnode = node.getElementsByTagName('scaleFactor').item(0);
      if ~isempty(elnode)
        scaleFac = parseParamString(model,robotnum,char(elnode.getAttribute('value')));
      end
      
      elnode = node.getElementsByTagName('limits').item(0);
      if ~isempty(elnode)
        limits = parseParamString(model,robotnum,char(elnode.getAttribute('value')));
      end
      
      if exist('limits')
        obj = RigidBodyThrust(parent, orig, dir, scaleFac, limits);
      else
        obj = RigidBodyThrust(parent, orig, dir, scaleFac);
      end
    end
  end
  
end
