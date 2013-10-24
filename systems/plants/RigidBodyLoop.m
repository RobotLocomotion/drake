classdef RigidBodyLoop < RigidBodyElement
  
  properties
    name
    body1
    pt1
    body2
    pt2
  end

  methods (Static)
    function pt = parseLink(node,options)
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0); 
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(str2num(char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(str2num(char(origin.getAttribute('rpy'))),3,1);
        end
      end

      pt = xyz;
    end
    
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
  end
  
end
