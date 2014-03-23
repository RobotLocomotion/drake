classdef RigidBodyGeometry

  methods % to be implemented in derived classes
    pts = getPoints(obj);  % returned in body coordinates
    lcmt_viewer_geometry_data = serializeToLCM(obj);
  end
  
  methods
    function pts = getPlanarPoints(obj,x_axis,y_axis,view_axis)
      % output is compatible with patch for 2D viewing
      % still returns a 3xn, but the z-axis is constant (just meant for
      % depth ordering in the 2d figure)
      
      Tview = [x_axis, y_axis, view_axis]';
      valuecheck(svd(Tview),[1;1;1]);  % assert that it's orthonormal
      
      pts = getPoints(obj);
      
      % project it into 2D frame
      pts = Tview*pts;
      
      % keep only convex hull (todo: do better here)
      ind = convhull(pts(1,:),pts(2,:));
      z = max(pts(3,:));
      
      % take it back out of view coordinates
      pts = Tview'*[pts(1:2,ind); repmat(z,1,length(ind))];
    end
    
    function writeWRLShape(obj,fp,td)
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

      tabprintf(fp,'Transform {\n'); td=td+1;
      wrlT = obj.T;
      if isa(obj,'RigidBodyCylinder')  
        % default axis for cylinder in urdf is the z-axis, but
        % the default in vrml is the y-axis.
        wrlT = wrlT*[1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1]; 
      end
      tabprintf(fp,'translation %f %f %f\n',wrlT(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(wrlT(1:3,1:3)));

      tabprintf(fp,'children Shape {\n'); td=td+1;
      tabprintf(fp,'%s\n',wrlGeometryStr(obj));
      tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      td=td-1; tabprintf(fp,'}\n');  % end Shape {
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end
  end
  
  methods (Static)
    function obj = parseURDFNode(node,x0,rpy,model,robotnum,options)
      T= [quat2rotmat(rpy2quat(rpy)),x0; 0 0 0 1];
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            size = parseParamString(model,robotnum,char(thisNode.getAttribute('size')));
            obj = RigidBodyBox(size);
          case 'sphere'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            obj = RigidBodySphere(r);
          case 'cylinder'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,robotnum,char(thisNode.getAttribute('length')));
            obj = RigidBodyCylinder(r,l);  % l/2
          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            
            if ~isempty(strfind(filename,'package://'))
              filename=strrep(filename,'package://','');
              [package,filename]=strtok(filename,filesep);
              filename=[rospack(package),filename];
            else
              [path,name,ext] = fileparts(filename);
              if (path(1)~=filesep)  % the it's a relative path
                path = fullfile(options.urdfpath,path);
              end
              filename = fullfile(path,[name,ext]);
            end
            
            obj = RigidBodyMesh(GetFullPath(filename));
        end
        obj.T = T;
      end
    end
  end
  
  properties
    T = eye(4);  % coordinate transform (from geometry to link coordinates)
    c = [.7 .7 .7];  % 3x1 color
  end
end
