classdef RigidBodyLoop
  
  properties
    name
    body1
    T1
    body2
    T2
    least_common_ancestor
  end

  methods (Static)
    
    
    function T = parseLink(node,options)
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

      error('still need to update this for the 3D case');
      T = [rotmat(theta),xyz;0,0,1];
    end
    
  end
end
