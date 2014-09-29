classdef RigidBodyHeightMapTerrain < RigidBodyTerrain & RigidBodyGeometry
  
  methods
    function obj = RigidBodyHeightMapTerrain(x,y,Z,terrain_to_world_transform,options)
      % @param terrain_to_world_transform a 4x4 transformation matrix.
      % default I
      
      if nargin<4, terrain_to_world_transform=eye(4); end
      
      obj@RigidBodyGeometry(4,terrain_to_world_transform);
      
      % note: I hate that these two are switched, but it is the convention
      % for meshes in matlab
      assert(size(Z,2)==length(x));
      assert(size(Z,1)==length(y));
      obj.x = x;
      obj.y = y;
      obj.Z = Z';  % store it in ndgrid format, instead of meshgrid format
      obj.T_world_to_terrain = inv(terrain_to_world_transform);
    end

    function pts = getPoints(obj)
      % returned in body coordinates
      [X,Y] = ndgrid(obj.x,obj.y);
      pts = obj.T(1:3,:)*[X(:)';Y(:)';obj.Z(:)';ones(1,numel(X))];
    end
    
    function pts = getBoundingBoxPoints(obj)
      % returned in body coordinates
      min_vals = [min(obj.x); min(obj.y); min(min(obj.Z))];
      max_vals = [max(obj.x); max(obj.y); max(max(obj.Z))];
      min_idx = logical([ 0 1 1 0 0 1 1 0;
                          1 1 1 1 0 0 0 0;
                          1 1 0 0 0 0 1 1]);
      pts = obj.T(1:3,:)*[repmat(min_vals,1,8).*min_idx + repmat(max_vals,1,8).*(~min_idx); ones(1,8)];
    end
    
    function shape = serializeToLCM(obj)
      fname = [tempname,'.obj'];
      writeOBJ(obj,fname);
      shape = drake.lcmt_viewer_geometry_data();
      shape.type = shape.MESH;
      shape.string_data = fname;
      shape.num_float_data = 1;
      shape.float_data = 1;  % scale

      shape.position = obj.T(1:3,4);
      shape.quaternion = rotmat2quat(obj.T(1:3,1:3));
      shape.color = [obj.c(:);1.0];
    end

    function geom = getRigidBodyContactGeometry(obj)
      geom = [];
    end

    function geom = getRigidBodyShapeGeometry(obj)
      geom = obj;
    end
    
    function plotTerrain(obj)
      [X,Y] = ndgrid(obj.x,obj.y);
      xyz = [X(:)';Y(:)';obj.Z(:)'];
      xyz = obj.T(1:3,:)*[xyz;1+0*xyz(1,:)];
      mesh(reshape(xyz(1,:),size(X)),reshape(xyz(2,:),size(X)),reshape(xyz(3,:),size(X)));
      xlabel('x');
      ylabel('y');
      zlabel('z');
      axis equal;
    end
    
    function [z,normal] = getHeight(obj,xy)
      [m,n] = size(xy);
      if (m>2) xy = xy(1:2,:); end
      
      % flip to terrain coordinates
      xy = homogTransMult(obj.T_world_to_terrain([1 2 4],[1 2 4]),xy);

      [indices,coefs,dcoefs] = barycentricInterpolation({obj.x,obj.y},xy);
      z = sum(obj.Z(indices).*coefs,1);
      if (nargout>1)
        fx = sum(obj.Z(indices).*reshape(dcoefs(:,1),size(indices,1),n));
        fy = sum(obj.Z(indices).*reshape(dcoefs(:,2),size(indices,1),n));
        normal = [-fx;-fy;ones(1,n)];
      end
    end
    
    function writeOBJ(obj,filename)
      % writes the mesh to an alias wavefront file (e.g. for the viewers to
      % parse)
      % adapted from http://www.aleph.se/Nada/Ray/saveobjmesh.m
      [x,y] = ndgrid(obj.x,obj.y);
      [~,normals]=getHeight(obj,[x(:)';y(:)']);
      nx = reshape(normals(1,:),size(x));
      ny = reshape(normals(2,:),size(x));
      nz = reshape(normals(3,:),size(x));
      
      l=size(x,1); h=size(x,2);

      n=zeros(l,h);

      [path,name,ext] = fileparts(filename);
      if isempty(ext), ext='.obj'; end
      filename = fullfile(path,[name,ext]);
      
      fid=fopen(filename,'w');

      n=reshape(1:(l*h),l,h);
      [i,j]=ndgrid(linspace(0,1,l),linspace(0,1,h));
      fprintf(fid,'v %f %f %f\n',[x(:)';y(:)';obj.Z(:)']);
      fprintf(fid,'vt %f %f\n',[i(:)';j(:)']);
      fprintf(fid,'g mesh\n');
      fprintf(fid,'f %d/%d %d/%d %d/%d %d/%d\n',[reshape(n(1:end-1,1:end-1),1,[]);reshape(n(1:end-1,1:end-1),1,[]);reshape(n(2:end,1:end-1),1,[]);reshape(n(2:end,1:end-1),1,[]);reshape(n(2:end,2:end),1,[]);reshape(n(2:end,2:end),1,[]);reshape(n(1:end-1,2:end),1,[]);reshape(n(1:end-1,2:end),1,[])]);
      fprintf(fid,'g\n\n');
      fclose(fid);
    end
    
    function writeWRLShape(obj,fp,td)
      
      fprintf(fp,'Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 1 0 0 } } }\n');
      fprintf(fp,'Transform { translation %f %f %f  children [ Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 0 0 1 } } } ] }\n',obj.x(end),obj.Z(end),-obj.y(end));
      fprintf(fp,'Transform { translation 1 0 0  children [ Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 0 1 0 } } } ] }\n');
      fprintf(fp,'Transform { translation 0 0 -1  children [ Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 0 1 0 } } } ] }\n');

      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end
      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'translation %f %f %f\n',obj.T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(obj.T(1:3,1:3)));
      tabprintf(fp,'children [\n'); td=td+1;

      xspacing = mean(diff(obj.x));
      yspacing = mean(diff(obj.y));
      if any(abs(diff(obj.x)-xspacing)>1e-5) || any(abs(diff(obj.x)-xspacing)>1e-5),
        error('Drake:RigidBodyHeightMapTerrain:NonUniformGrid','vrml export currently only support uniform grids');
      end
      [m,n]=size(obj.Z);

      % todo: use the color field
      %  color1 = [204 102 0]/256;  % csail orange
      color1 = hex2dec({'ee','cb','ad'})/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      color2 = hex2dec({'cd','af','95'})/256;

      % vrml view axis is y up, and elevation grid is y=height(x,z)
      % so I need to treat y=z, and z=-y
      fprintf(fp,'Transform {\n  translation %f %f %f\n children [\n',obj.x(1),obj.y(1),0);
      fprintf(fp,'Transform {\n  rotation  1 0 0 1.5708\n scale 1 1 1\n children [\n');

      fprintf(fp,'Shape { geometry ElevationGrid {\n');
      %        fprintf(fp,'  solid "false"\n');
      fprintf(fp,'  xSpacing %f\n',xspacing);
      fprintf(fp,'  zSpacing %f\n',-yspacing);
      fprintf(fp,'  xDimension %d\n',m);
      fprintf(fp,'  zDimension %d\n',n);
      fprintf(fp,'  height [');
      fprintf(fp,' %d', obj.Z);
      fprintf(fp,' ]\n');
      fprintf(fp,'  solid TRUE\n');
      fprintf(fp,'  colorPerVertex TRUE\n');
      % note: colorPerVertex FALSE seems to be unsupported (loads but renders incorrectly) in the default simulink 3D animation vrml viewer
      fprintf(fp,'   color Color { color [');
      for i=1:m
        for j=1:n
          if rem(i+j,2)==0
            fprintf(fp,' %.1f %.1f %.1f,', color1);
          else
            fprintf(fp,' %.1f %.1f %.1f,', color2);
          end
        end
      end
      fprintf(fp,'] }\n'); % end Color
      %  [nx,ny,nz] = surfnorm(obj.terrain_height);
%      fprintf(fp,'  normalPerVertex TRUE\n');
%      fprintf(fp,'  normal Normal { vector [');
%      fprintf(fp,' %.2f %.2f %.2f,', [0, -1, 0]); %[nx(:),-nz(:),ny(:)]');  % rotx(pi/2)*normal
%      fprintf(fp,'] }\n'); % end normal
      fprintf(fp,'}\n}\n'); % end Shape
      fprintf(fp,']\n}\n'); % end Transform
      fprintf(fp,']\n}\n'); % end Transform
      
      td=td-1; tabprintf(fp,']\n'); % end children
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
      
      fprintf(fp,'\n\n');
    end
  end
  
  methods (Static=true)
    function obj = loadFromImage(filename,ax,terrain_to_world_transform,options)
      % @param filename the path to the image file
      % @param ax axis specification [xmin,xmax,ymin,ymax].  note that the
      % top left pixel of the image will be at xmin,ymax
      % @param terrain_to_world_transform a 4x4 transformation matrix
      % @option z_scale grayscale 255 = z_scale meters.  @default 1
      % for additional options, see the class constructor;
      
      if nargin<3, terrain_to_world_transform=eye(4); end
      if nargin<4, options=struct(); end
      
      if ~isfield(options,'z_scale'), options.z_scale=1; end
        
      a=imread(filename);
      Z=options.z_scale*double(rgb2gray(a))/255;
      Z=flipud(Z); % flip the y axis to go from image coordinates to cartesian coordinates
      x = linspace(ax(1),ax(2),size(a,1));
      y = linspace(ax(3),ax(4),size(a,2));
      obj = RigidBodyHeightMapTerrain(x,y,Z,terrain_to_world_transform,options);
    end
    
  end
  
  properties (SetAccess=protected)
    x,y,Z;  % heightmap data
    T_world_to_terrain;
  end
end