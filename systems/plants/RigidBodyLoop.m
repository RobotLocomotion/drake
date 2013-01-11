classdef RigidBodyLoop
  
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
end
