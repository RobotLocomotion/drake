classdef PlanarRigidBodyLoop < RigidBodyLoop
  
  properties 
    jcode
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
      
      if any(rpy)
        rpya=rpy2axis(rpy); rpyangle=rpya(4); rpyaxis=rpya(1:3);
        if abs(dot(rpyaxis,options.view_axis))<(1-1e-6)
          error('joints out of the plane are not supported');
          % note that if they were, it would change the way that I have to 
          % parse geometries, inertias, etc, for all of the children.
        elseif dot(rpyaxis,options.view_axis)<0
          rpyangle=-rpyangle;
        end
      else
        rpyangle=0;
      end
      
      pt = [options.x_axis'; options.y_axis']*xyz;
    end
    
  end
end