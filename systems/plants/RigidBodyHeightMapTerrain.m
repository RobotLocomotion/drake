classdef RigidBodyHeightMapTerrain < RigidBodyTerrain
  
  methods
    function obj = RigidBodyHeightMapTerrain(terrain_imagefile_or_heightmatrix,terrain_to_world_transform,varargin)
      % usage:
      %   RigidBodyHeightMapTerrain(terrain,T)
      %  or
      %   RigidBodyHeightMapTerrain(obj,terrain,pos,size)
      
      if ischar(terrain_imagefile_or_heightmatrix)
        a=imread(terrain_imagefile_or_heightmatrix);
        terrain_height=double(rgb2gray(a))/255;
      else
        terrain_height = terrain_imagefile_or_heightmatrix;
      end
      
      [m,n]=size(terrain_height);
      if (nargin<2 || isempty(terrain_to_world_transform))
        terrain_to_world_transform = [1,0,0,-(m-1)/2;0,1,0,(n-1)/2;0,0,1,0;0,0,0,1]; 
      else
        if sizecheck(terrain_to_world_transform,3)
          % then it's setTerrain(obj,terrain,pos,size)
          terrain_pos = terrain_to_world_transform(:);
          if (nargin>2)
            terrain_size=varargin{1}(:);
            sizecheck(terrain_size,[3 1]);
          else
            terrain_size=[m-1;n-1;max(max(abs(terrain_height)))];
          end
          terrain_to_world_transform = [diag(terrain_size./[m-1;n-1;max(max(abs(terrain_height)))]), terrain_pos; 0 0 0 1]; 
        end

        sizecheck(terrain_to_world_transform,[4 4]);
      end
      
      obj.terrain_height = terrain_height;
      obj.T_terrain_to_world = terrain_to_world_transform;
      obj.T_world_to_terrain = inv(terrain_to_world_transform);
    end
    
    function [z,normal] = getHeight(obj,xy)
      [m,n] = size(xy);
      if (m>2) xy = xy(1:2,:); end
      
      % flip to terrain coordinates
      flipY = diag([1,-1,1]);
      xy = homogTransMult(flipY*obj.T_world_to_terrain([1 2 4],[1 2 4]),xy);

      %   ^ y
      %  c|
      %   |
      %   |
      %    -------> x
      %   a     b
      % a,b,c are all (effectively) integers
      a = floor(xy);
      b = a; b(1,:)=b(1,:)+1;
      c = a; c(2,:)=c(2,:)+1;

      %   ^ y
      %  c|\  |a    if beyond the diagonal, then flip a to the opposite corner
      %   | \*|
      %   |  \|
      %    -------> x
      %       b
      tmp = xy(1:2,:) - c;
      to_flip = -tmp(1,:)<tmp(2,:);
      a(:,to_flip) = a(:,to_flip)+1; % same as + repmat([1;1],1,sum(to_flip));

      % compute barycentric coordinates
      % http://en.wikipedia.org/wiki/Barycentric_coordinate_system_%28mathematics%29
      % with r1=a,r2=b,r3=c
      T_noflip = [-1 -1; 1 0];
      T_flip = [1 1; 0 -1];
      lambda=a;
      lambda(:,~to_flip) = T_noflip*tmp(:,~to_flip);
      lambda(:,to_flip) = T_flip*tmp(:,to_flip);
      lambda(3,:) = 1-lambda(1,:)-lambda(2,:);

      % todo: handle case where i'm off the mesh?  (with z = -inf?)
      % if it happens now, the code below will error with "bad index"
      s = size(obj.terrain_height);
      az = obj.terrain_height(sub2ind(s,a(1,:)+1,a(2,:)+1));
      bz = obj.terrain_height(sub2ind(s,b(1,:)+1,b(2,:)+1));
      cz = obj.terrain_height(sub2ind(s,c(1,:)+1,c(2,:)+1));

      z = lambda(1,:).*az + lambda(2,:).*bz + lambda(3,:).*cz;

      normal = cross([b;bz]-[a;az],[c;cz]-[a;az]);  %[zeros(2,n); ones(1,n)];
      normal(:,to_flip) = -normal(:,to_flip);

      % convert back to world coordinates
      flipY = diag([1,-1,1,1]);
      pos = [xy;z];
      pos = homogTransMult(obj.T_terrain_to_world*flipY,pos);
      z = pos(3,:);
      normal = homogTransMult(obj.T_terrain_to_world*flipY,normal)-repmat(homogTransMult(obj.T_terrain_to_world,zeros(3,1)),1,n);

      % normalize normals
      normal = normal./repmat(sqrt(sum(normal.^2,1)),3,1);
    end
    
    function [xgv,ygv] = writeWRL(obj,fp) % for visualization
      T = obj.T_terrain_to_world;
      xyz = homogTransMult(T,[0;0;0]);
      axisangle = rotmat2axis(T(1:3,1:3));
      xspacing = norm(homogTransMult(T,[1;0;0])-homogTransMult(T,[0;0;0]));
      yspacing = norm(homogTransMult(T,[0;1;0])-homogTransMult(T,[0;0;0]));
      [m,n]=size(obj.terrain_height);
      [X,Y]=meshgrid(0:(m-1),0:(n-1));
      pts = homogTransMult(T,[X(:),Y(:),obj.terrain_height(:)]');
  
      %  color1 = [204 102 0]/256;  % csail orange
      color1 = hex2dec({'ee','cb','ad'})/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      color2 = hex2dec({'cd','af','95'})/256;
      fprintf(fp,'Transform {\n  translation %f %f %f\n  rotation %f %f %f %f\n  children [\n',xyz(1),xyz(2),xyz(3),axisangle(1),axisangle(2),axisangle(3),axisangle(4));
      fprintf(fp,'Transform {\n  rotation  1 0 0 1.5708\n  children [\n');
      fprintf(fp,'Shape { geometry ElevationGrid {\n');
      %        fprintf(fp,'  solid "false"\n');
      fprintf(fp,'  xSpacing %f\n',xspacing);
      fprintf(fp,'  zSpacing %f\n',yspacing);
      fprintf(fp,'  xDimension %d\n',m);
      fprintf(fp,'  zDimension %d\n',n);
      fprintf(fp,'  height [');
      fprintf(fp,' %d', pts(3,:));
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
      %  fprintf(fp,'  normalPerVertex TRUE\n');
      %  fprintf(fp,'  normal Normal { vector [');
      %  fprintf(fp,' %.2f %.2f %.2f,', [nx(:),-nz(:),ny(:)]');  % rotx(pi/2)*normal
      %  fprintf(fp,'] }\n'); % end normal
      fprintf(fp,'}\n}\n'); % end Shape
      fprintf(fp,']\n}\n'); % end Transform
      fprintf(fp,']\n}\n'); % end Transform
      fprintf(fp,'\n\n');
    end
  end
  
  properties
    terrain_height=zeros(20);  % height(i,j) = height at (xspacing * (i-1),-yspacing * (j-1))
    T_terrain_to_world = [1,0,0,-10;0,1,0,10;0,0,1,0;0,0,0,1];  % translation from terrain coordinates to world coordinates
    T_world_to_terrain;
  end
end