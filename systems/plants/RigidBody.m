classdef RigidBody < handle
  
  properties
    % link properties
    linkname=''  % name of the associated link
    I=[];  % spatial mass/inertia
    geometry={}  % geometry (compatible w/ patch).  see parseVisual below.
    wrlgeometry=''; % geometry (compatible w/ wrl).  see parseVisual below.
    dofnum=0     % the index in the state vector corresponding to this joint
    ground_contact=[];  % a 2xn matrix with [x;z] positions of contact points
    
    % joint properties
    jointname=''
    parent
    pitch=[];        % for featherstone 3D models
    jcode=-1;        % for featherstone planar models
    Xtree=[];   % velocity space coordinate transform *from parent to this node*
    XtreeForChildren=[];
    Ttree=[];   % position space coordinate transform *from this node to parent*
    wrljoint='';  % tranformation to joint coordinates in wrl syntax
    damping=0
    
    % dynamic properties (e.g. state at runtime)
    T  % transformation from this body to world coordinates
%    X  % motion transform *from world coordinates to to this node*
%    v  % velocity [omega; v] of this body *in this body coordinates*
%    v  % velocity [omega; v] of this body in world coordinates
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
      inertia = eye(3);
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
        if inode.hasAttribute('ixx'), I(1,1)=str2num(char(inode.getAttribute('ixx'))); end
        if inode.hasAttribute('ixy'), I(1,2)=str2num(char(inode.getAttribute('ixy'))); I(2,1)=I(1,2); end
        if inode.hasAttribute('ixz'), I(1,3)=str2num(char(inode.getAttribute('ixz'))); I(3,1)=I(1,3); end
        if inode.hasAttribute('iyy'), I(2,2)=str2num(char(inode.getAttribute('iyy'))); end
        if inode.hasAttribute('iyz'), I(2,3)=str2num(char(inode.getAttribute('iyz'))); I(3,2)=I(2,3); end
        if inode.hasAttribute('izz'), I(3,3)=str2num(char(inode.getAttribute('izz'))); end
      end      

      if (options.twoD)
        if any(rpy)
          error('rpy in inertia block not implemented yet (but would be easy)');
        end
        body.I = mcIp(mass,xyz([1 3]),I(2,2));
      else
        if any(rpy)
          error('rpy in inertia block not implemented yet (but would be easy)');
        end
        body.I = mcI(mass,xyz,I);
      end
        
    end

    function body = parseVisual(body,node,model,options)
      
      x0=zeros(3,1);
      rpy=zeros(3,1);
      xpts = [];
      zpts = [];
      c = .7*[1 1 1];
      
      wrl_transform_str = '';
      wrl_shape_str='';
      wrl_appearance_str='';
      
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
                  wrl_transform_str = sprintf('%s\ttranslation %f %f %f\n',wrl_transform_str,x0(1),x0(2),x0(3));
                case 'rpy'
                  rpy=str2num(char(thisNode.getAttribute('rpy')));
                  wrl_transform_str = [wrl_transform_str,'\trotation',sprintf(' %f %f %f %f\n',rpy2axis(rpy))];
              end
            end
          case {'geometry','#text','#comment'}
            % intentionally fall through
          case 'material'
            c = parseMaterial(model,thisNode,options);
            wrl_appearance_str = sprintf('appearance Appearance { material Material { diffuseColor %f %f %f } }\n',c(1),c(2),c(3)); 
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual.']);
        end
      end
      
      if ~isempty(wrl_transform_str)
        if isempty(body.linkname)
          wrl_transform_str = ['Transform {\n',wrl_transform_str];
        else
          wrl_transform_str = [sprintf('DEF %s Transform {\n',body.linkname),wrl_transform_str];
        end
        wrl_transform_str = [wrl_transform_str,'\tchildren [\n'];
      end
      
      % second pass: through and handle the geometry (after the
      % coordinates)
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'geometry'
            if (options.twoD)
              if (~isempty(xpts)) error('multiple geometries not handled yet (but would be trivial)'); end
              [xpts,zpts] = RigidBody.parseGeometry(thisNode,x0,rpy,options);
            end
            wrl_shape_str = [wrl_shape_str,RigidBody.parseWRLGeometry(thisNode,wrl_appearance_str)];
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
      
      if isempty(wrl_transform_str)
        body.wrlgeometry = wrl_shape_str;
      else
        body.wrlgeometry = [wrl_transform_str,wrl_shape_str,'\t]\n}'];
      end
    end
    
    function body = parseCollision(body,node,options)
      x0 = zeros(3,1);
      rpy = zeros(3,1);
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
      
      if options.twoD  % todo: implement collisions in 3D
        % note: could support multiple geometry elements
        geomnode = node.getElementsByTagName('geometry').item(0);
        if ~isempty(geomnode)
          [xpts,zpts] = RigidBody.parseGeometry(geomnode,xyz,rpy,options);
          body.ground_contact=[xpts; zpts];
        end
      end
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
    
    function writeWRLJoint(body,fp)
      if isempty(body.jointname)
        fprintf(fp,'Transform {\n');
      else
        fprintf(fp,'DEF %s Transform {\n',body.jointname); 
      end
      if (body.jcode==1) % then it's a pin joint (only allowed about the y axis, so far
        fprintf(fp,'rotation 0 1 0 0\n'); 
      elseif (body.jcode==2) % then it's a slider
        fprintf(fp,'translation 0 0 0\n');
      end
    end
    
    function td=writeWRLBody(body,fp,td)
      t=''; 
      for i=1:td, t=[t,'\t']; end
      s = regexprep(body.wrlgeometry,'\n',['\n',t]);
      fprintf(fp,[t,s,'\n']);
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
              warning('full cylinder geometry not implemented yet');
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

    function wrlstr = parseWRLGeometry(node,wrl_appearance_str)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      
      wrlstr='';
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = str2num(char(thisNode.getAttribute('size')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Box { size %f %f %f }\n\t%s}\n',s(1),s(2),s(3),wrl_appearance_str)];
            
          case 'cylinder'
            r = str2num(char(thisNode.getAttribute('radius')));
            l = str2num(char(thisNode.getAttribute('length')));
            
            % default axis for cylinder in urdf is the z-axis, but
            % the default in vrml is the y-axis. 
            wrlstr=[wrlstr,sprintf('Transform {\n\trotation 1 0 0 1.5708\n\tchildren Shape {\n\t\tgeometry Cylinder {\n\t\t\theight %f\n\t\t\tradius %f\n\t\t}\n\t\t%s\n\t}\n}\n',l,r,wrl_appearance_str)];
            
          case 'sphere'
            r = str2num(char(thisNode.getAttribute('radius')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Sphere { radius %f }\n\t%s}\n',r,wrl_appearance_str)];

          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
        end
      end
      
    end
    
  end
end
