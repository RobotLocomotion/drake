classdef RigidBodyLoop < RigidBodyElement
  
  properties
    name
    body1
    pt1=[0;0;0]
    body2
    pt2=[0;0;0]
    axis=[1;0;0]
  end
  
  methods    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body1 = map_from_old_to_new(obj.body1);
      obj.body2 = map_from_old_to_new(obj.body2);
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.body1 == body_ind)
        obj.pt1 = model.body(body_ind).Ttree(1:end-1,:)*[obj.pt1;1];
        obj.body1 = model.body(body_ind).parent;
      end
      if (obj.body2 == body_ind)
        obj.pt2 = model.body(body_ind).Ttree(1:end-1,:)*[obj.pt2;1];
        obj.body2 = model.body(body_ind).parent;
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('need to implement this.  (see changeRootLink)');
    end
    
  end
  
end
