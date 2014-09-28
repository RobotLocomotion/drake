classdef RigidBodyHeightMapTerrain < RigidBodyTerrain
  
  methods
    function obj = RigidBodyHeightMapTerrain(x,y,Z,terrain_to_world_transform,options)
      % @param terrain_to_world_transform a 4x4 transformation matrix.
      % default I
      
      if nargin<4, terrain_to_world_transform=eye(4); end
      
      % note: I hate that these two are switched, but it is the convention
      % for meshes in matlab
      assert(size(Z,2)==length(x));
      assert(size(Z,1)==length(y));
      obj.x = x;
      obj.y = y;
      obj.Z = Z;

      obj.T_terrain_to_world = terrain_to_world_transform;
      obj.T_world_to_terrain = inv(terrain_to_world_transform);

      fname = [tempname,'.obj'];
      writeOBJ(obj,fname);
      obj.geom = RigidBodyMesh(fname,obj.T_terrain_to_world);
    end
    
    function geom = getRigidBodyContactGeometry(obj)
      geom = [];
    end

    function geom = getRigidBodyShapeGeometry(obj)
      geom = obj.geom;
    end
    
    function plotTerrain(obj)
      [X,Y] = meshgrid(obj.x,obj.y);
      xyz = [X(:)';Y(:)';obj.Z(:)'];
      xyz = obj.T_terrain_to_world(1:3,:)*[xyz;1+0*xyz(1,:)];
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
      [x,y] = meshgrid(obj.x,obj.y);
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
    
    function [xgv,ygv] = writeWRL(obj,fp) % for visualization
      T = obj.T_terrain_to_world;
      xyz_min = homogTransMult(T,[obj.x(1);obj.y(1);0]);
      axisangle = rotmat2axis(T(1:3,1:3));
      xspacing = mean(diff(obj.x));
      yspacing = mean(diff(obj.y));
      if any(abs(diff(obj.x)-xspacing)>1e-4) || any(abs(diff(obj.x)-xspacing)>1e-4),
        error('Drake:RigidBodyHeightMapTerrain:NoUniformGrid','vrml export currently only support uniform grids');
      end
      [m,n]=size(obj.Z);
      
      error('need to update this');
      
      fprintf(fp,'Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 1 0 0 } } }\n');
      fprintf(fp,'Transform { translation %f %f %f  children [ Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 0 0 1 } } } ] }\n',obj.x(end),obj.Z(end),-obj.y(end));
      fprintf(fp,'Transform { translation 1 0 0  children [ Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 0 1 0 } } } ] }\n');
      fprintf(fp,'Transform { translation 0 0 -1  children [ Shape { geometry Sphere { radius .05 }\n appearance Appearance { material Material { diffuseColor 0 1 0 } } } ] }\n');
      
      %  color1 = [204 102 0]/256;  % csail orange
      color1 = hex2dec({'ee','cb','ad'})/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      color2 = hex2dec({'cd','af','95'})/256;

      % vrml view axis is y up, and elevation grid is y=height(x,z)
      % so I need to treat y=z, and z=-y
%      fprintf(fp,'Transform {\n  rotation  1 0 0 1.5708\n scale 1 1 1\n children [\n');

      fprintf(fp,'Transform {\n  translation %f %f %f\n  rotation %f %f %f %f\n  children [\n',xyz_min(1),0,xyz_min(2),axisangle(1),axisangle(3),-axisangle(2),axisangle(4));
      fprintf(fp,'Shape { geometry ElevationGrid {\n');
      %        fprintf(fp,'  solid "false"\n');
      fprintf(fp,'  xSpacing %f\n',xspacing);
      fprintf(fp,'  zSpacing %f\n',-yspacing);
      fprintf(fp,'  xDimension %d\n',m);
      fprintf(fp,'  zDimension %d\n',n);
      fprintf(fp,'  height [');
      fprintf(fp,' %d', obj.Z');
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
%      fprintf(fp,']\n}\n'); % end Transform
      fprintf(fp,']\n}\n'); % end Transform
      fprintf(fp,'\n\n');
    end
  end
  
  methods (Static=true)
    function obj = loadFromImage(filename,ax,terrain_to_world_transform,options)
      % @param filename the path to the image file
      % @param ax axis specification [xmin,xmax,ymin,ymax].  note that the
      % top left pixel of the image will be at xmin,ymax
      % @param terrain_to_world_transform a 4x4 transformation matrix
      % for options, see the class constructor;
      
      if nargin<3, terrain_to_world_transform=eye(4); end
      if nargin<4, options=struct(); end
      
      a=imread(filename);
      a=flipup(a); % flip the y axis to go from image coordinates to cartesian coordinates
      terrain_height=double(rgb2gray(a))/255;
      x = linspace(ax(1),ax(2),size(a,1));
      y = linspace(ax(3),ax(4),size(a,2));
      obj = RigidBodyHeightMapTerrain(x,y,Z,terrain_to_world_transform,options);
    end
    
  end
  
  properties (SetAccess=protected)
    x,y,Z;  % heightmap data
    T_terrain_to_world = [1,0,0,-10;0,1,0,10;0,0,1,0;0,0,0,1];  % translation from terrain coordinates to world coordinates
    T_world_to_terrain;
    geom;
  end
end