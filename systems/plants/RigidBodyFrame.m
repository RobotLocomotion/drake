classdef RigidBodyFrame
  % A number of RigidBodyElements (e.g. sensors, actuators) require 
  % their own Cartesian frame relative to a RigidBody's frame.  This
  % class provides the common utilities for this
  
  methods
    function obj = RigidBodyFrame(body_ind,xyz,rpy,name)
      sizecheck(body_ind,1);
      sizecheck(xyz,[3 1]);
      sizecheck(rpy,[3 1]);
      obj.body_ind = body_ind;
      obj.T = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1];
      obj.Ti = inv(obj.T);
      if nargin>3
        typecheck(name,'char');
        obj.name = name;
      end
    end
    
    function [x,J] = forwardKin(obj,model,kinsol,pts) % todo: support rotation_type?
      % convert pts from this frame to world frame
      pts = obj.T(1:3,:)*[pts; ones(1,size(pts,2))];
      if (nargout>1)
        [x,J] = forwardKin(model,kinsol,obj.body_ind,pts);
      else
        x = forwardKin(model,kinsol,obj.body_ind,pts);
      end
    end
    
    function x = frameKin(obj,model,kinsol,pts)
      % convert pts from world frame to this frame
      x = bodyKin(model,kinsol,obj.body_ind,pts);
      x = obj.Ti(1:3,:)*[x; 1 + 0*x(1,:)];
    end
    
    function x = bodyToFrame(obj,pts)
      x = obj.Ti(1:3,:)*[pts; 1 + 0*pts(1,:)];
    end
    
    function x = frameToBody(obj,pts)
      x = obj.T(1:3,:)*[pts; 1 + 0*pts(1,:)];
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body_ind = map_from_old_to_new(obj.body_ind);
    end    
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.body_ind == body_ind)
        obj.T = model.body(body_ind).Ttree*obj.T;
        obj.Ti = inv(obj.T);
        obj.body_ind = model.body(body_ind).parent;
      end
    end
  end

  properties
    name
    body_ind
    T
    Ti
  end
end

