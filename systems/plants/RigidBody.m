classdef RigidBody < handle
  
  properties
    % link properties
    linkname=''  % name of the associated link
    I=zeros(3);  % spatial mass/inertia
    geometry={}  % geometry (compatible w/ patch).  see parseVisual below.
    dofnum=0     % the index in the state vector corresponding to this joint
    ground_contact=[];  % a 2xn matrix with [x;z] positions of contact points
    
    % joint properties
    jointname=''
    parent
    jcode=-1
    Xtree=eye(3);   % velocity space coordinate transform *from parent to this node*
    Ttree=eye(3);   % position space coordinate transform *from this node to parent*
    damping=0
    
    % dynamic properties (e.g. state at runtime)
    T  % transformation from this body to world coordinates
%    X  % motion transform *from world coordinates to to this node*
%    v  % velocity [omega; v] of this body *in this body coordinates*
    v  % velocity [omega; v] of this body in world coordinates
  end
  
  methods
    function newbody = copy(body)
      % makes a shallow copy of the body
      % note that this functionality is in matlab.mixin.Copyable in newer
      % versions of matlab, but I've done it myself since i want to support
      % < v2011
      
      newbody=RigidBody();
      p=properties(body);
      for i=1:length(p)
        eval(['newbody.',p{i},'=body.',p{i}]);
      end
    end
    
    function body=parseInertial(body,node,options)
      mass = 0;
      com = zeros(2,1);
      theta = 0;
      inertia = 1;

      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'origin'
            at = thisNode.getAttributes();
            for j=1:at.getLength()
              thisAt = at.item(j-1);
              switch (lower(char(thisAt.getName())))
                case 'xyz'
                  com = reshape(str2num(char(thisAt.getValue())),3,1);
                  com = com([1 3]); % ignore y
                case 'rpy'
                  rpy=str2num(char(thisNode.getAttribute('rpy')));
                  theta = rpy(2);
                  if (any(rpy([1 3])))
                    warning([obj.linkname{linknum} ,' has an inertial block with out-of-plane rotations.  these will be ignored.']);
                  end
              end
            end
          case 'mass'
            mass = str2num(char(thisNode.getAttribute('value')));
          case 'inertia'
            at = thisNode.getAttributes();
            for j=1:at.getLength()
              thisAt = at.item(j-1);
              switch (lower(char(thisAt.getName())))
                case 'iyy'
                  inertia = str2num(char(thisAt.getValue()));
                case {'ixx','ixy','ixz','iyz','izz'}
                  % ignore y
              end
            end
          case {'#text','#comment'}
            % intentionally empty. ok to skip these quietly.
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/inertial.']);
        end
      end
      body.I=mcIp(mass,com,inertia);
    end

    function body = parseVisual(body,node,model,options)
      
      x0=zeros(3,1);
      rpy=zeros(3,1);
      xpts = [];
      zpts = [];
      c = .7*[1 1 1];
      
      % first pass:
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'origin'
            at = thisNode.getAttributes();
            for j=1:at.getLength()
              thisAt = at.item(j-1);
              switch (lower(char(thisAt.getName())))
                case 'xyz'
                  x0 = reshape(str2num(char(thisAt.getValue())),3,1);
                case 'rpy'
                  rpy=str2num(char(thisNode.getAttribute('rpy')));
              end
            end
          case {'geometry','#text','#comment'}
            % intentionally fall through
          case 'material'
            c = parseMaterial(model,thisNode,options);
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual.']);
        end
      end
      
      % second pass: through and handle the geometry (after the
      % coordinates)
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'geometry'
            if (~isempty(xpts)) error('multiple geometries not handled yet (but would be trivial)'); end
            [xpts,zpts] = RigidBody.parseGeometry(thisNode,x0,rpy,options);
        end
      end
      
      % % useful for testing local geometry
      % h=patch(xpts,zpts,.7*[1 1 1]);
      % axis equal
      % pause;
      % delete(h);
      
      body.geometry{1}.x = xpts;
      body.geometry{1}.z = zpts;
      body.geometry{1}.c = c;
      
    end
    
    function body = parseCollision(body,node,options)
      x0 = zeros(3,1);
      rpy = zeros(3,1);
      xpts = [];
      zpts = [];
      
      % first pass:
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'origin'
            at = thisNode.getAttributes();
            for j=1:at.getLength()
              thisAt = at.item(j-1);
              switch (lower(char(thisAt.getName())))
                case 'xyz'
                  x0 = reshape(str2num(char(thisAt.getValue())),3,1);
                case 'rpy'
                  rpy=str2num(char(thisNode.getAttribute('rpy')));
              end
            end
          case {'geometry','#text','#comment'}
            % intentionally fall through
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/collision.']);
        end
      end
      
      % second pass: through and handle the geometry (after the
      % coordinates)
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'geometry'
            if (~isempty(xpts)) error('multiple geometries not handled yet (but would be trivial)'); end
            [xpts,zpts] = RigidBody.parseGeometry(thisNode,x0,rpy,options);
        end
      end
      
      body.ground_contact = [xpts; zpts];

    end
      
    function b=leastCommonAncestor(body1,body2)
      % recursively searches for the lowest body in the tree that is an
      % ancestor to both body1 and body2

      b=body2;
      if (body1==body2) return; end
      
      % check if body1 is an ancestor to body2
      while (~isempty(b.parent))
        b=b.parent;
        if (body1==b) return; end
      end
      
      % body1 is not an ancestor to body2.  check body1's parent (and
      % recurse)
      b = leastCommonAncestor(body1.parent,body2);
    end
    
  end
  
  methods (Static)
    function [x,z] = parseGeometry(node,x0,rpy,options)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      % option twoD true implies that I can safely ignore y.
      x=[];z=[];
      T = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),x0]; % intentially leave off the bottom row [0,0,0,1];
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        cx=[]; cy=[]; cz=[];
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = str2num(char(thisNode.getAttribute('size')));
            
            cx = s(1)/2*[-1 1 1 -1 -1 1 1 -1];
            cy = s(2)/2*[1 1 1 1 -1 -1 -1 -1];
            cz = s(3)/2*[1 1 -1 -1 -1 -1 1 1];
            
          case 'cylinder'
            r = str2num(char(thisNode.getAttribute('radius')));
            l = str2num(char(thisNode.getAttribute('length')));
            
            if (options.twoD && rpy(1)==0) % then it just looks like a box
              cx = r*[-1 1 1 -1];
              cy = [0 0 0 0];
              cz = l/2*[1 1 -1 -1];
            elseif (options.twoD && abs(mod(rpy(1),pi)-pi/2)<1e-4) % then it just looks like a circle
              error('not implemented yet');
            else  % full cylinder geometry
              error('not implemented yet');
            end
            
          case 'sphere'
            r = str2num(char(thisNode.getAttribute('radius')));
            if (r==0)
              cx=0; cy=0; cz=0;
            else
              theta = 0:.1:2*pi;
              cx = r*cos(theta);
              cy = 0*theta;
              cz = r*sin(theta);
            end
          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
        end
        npts = size(cx,2);
        if (npts>0)
          % transform into this body's coordinates
          pts = T*[cx;cy;cz;ones(1,npts)];
          
          if (~isempty(x)) error('multiple geometries not handled yet (but would be trivial)'); end
          
          % convex hull?
          if (npts>1) 
            i = convhull(pts(1,:),pts(3,:)); 
          else
            i=1;
          end
          x=pts(1,i)';z=pts(3,i)';
        end
      end
      
    end
  end
end
