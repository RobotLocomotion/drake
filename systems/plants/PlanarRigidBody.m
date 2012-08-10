classdef PlanarRigidBody < RigidBody
  
  properties
    geometry={}  % geometry (compatible w/ patch).  see parseVisual below.
    jcode=-1;        % for featherstone planar models
  end
  
  methods
    function obj = PlanarRigidBody()
      obj = obj@RigidBody();
      obj.I = zeros(3);
      obj.Xtree = eye(3);
      obj.Ttree = eye(3);
    end
    
    function body=parseInertial(body,node,options)
      mass = 0;
      I = 1;
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(str2num(char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(str2num(char(origin.getAttribute('rpy'))),3,1);
        end
      end
      massnode = node.getElementsByTagName('mass').item(0);
      if ~isempty(massnode)
        if (massnode.hasAttribute('value'))
          mass = str2num(char(massnode.getAttribute('value')));
        end
      end
      inode = node.getElementsByTagName('inertia').item(0);
      if ~isempty(inode)
        if inode.hasAttribute('iyy'), I=str2num(char(inode.getAttribute('iyy'))); end
      end      

      if any(rpy)
        error('rpy in inertia block not implemented yet (but would be easy)');
      end
      body.I = mcIp(mass,xyz([1 3]),I);
    end    
    
  end
  
end