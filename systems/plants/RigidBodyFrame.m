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
      if nargin>3
        typecheck(name,'char');
        obj.name = name;
      end
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body_ind = map_from_old_to_new(obj.body_ind);
    end    
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.body_ind == body_ind)
        obj.T = model.body(body_ind).Ttree*obj.T;
        obj.body_ind = model.body(body_ind).parent;
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      if (obj.body_ind == body_ind)
        obj.T = obj.T*T_old_body_to_new_body;
      end
    end
    
  end

  properties
    name
    body_ind
    T
  end
end

