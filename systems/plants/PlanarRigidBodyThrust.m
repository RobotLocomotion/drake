classdef PlanarRigidBodyThrust < RigidBodyForceElement

  properties
    bodyind    % RigidBody index of the parent
    origin=zeros(2,1);  %xz of the reference point for the thrust element
    %angle in radians that the thrust is angled away from the parent body's
    %x-axis (rotation around the y-axis)
    direction = 0 
    scaleFactor = 1; %amount to scale the input by.
    input_num = 0;
    limits = [-inf inf];
  end
  
  methods
    function obj = PlanarRigidBodyThrust(parent_body, orig, dir, scaleFac, limits)
        if ~isa(parent_body, 'numeric');
            error('Drake:RigidBodyThrust:InvalidParent','Force Type Thrust does not have a proper RigidBody parent');
        end
        obj.bodyind = parent_body;
        obj.origin = orig;
        dir = dir/norm(dir);
        obj.direction = reshape(dir,2,1);
        obj.direct_feedthrough_flag = true;
        if (nargin > 3)
            obj.scaleFactor = scaleFac;
        end
        if (nargin > 4)
            obj.limits = limits;
        end
    end %constructor
    
    function [force, B_mod] = computeSpatialForce(obj,manip,q,qd)
        %B_mod maps the input to generalized forces.
        kinsol = doKinematics(manip,q, false, false);
        %origin = [x y z] of the reference point
        [x,J] = forwardKin(manip,kinsol,obj.bodyind,obj.origin(1:2)', 0);
        B_mod = manip.B*0; %initialize B_mod
        force = sparse(3,getNumBodies(manip));
        %J expects the force to be in world coordinates
        kinsolT = kinsol.T{obj.bodyind};
        direction_world = kinsolT(1:2,1:2)*obj.direction;
        %B_mod(:,obj.input_num) = J'*obj.direction*obj.scaleFactor;
        B_mod(:,obj.input_num) = J'*direction_world*obj.scaleFactor;
        B_mod;
    end
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.bodyind = map_from_old_to_new(obj.bodyind);
    end
    
  end
  
end
