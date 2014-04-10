classdef RigidBodyFlatTerrain < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere
  
  methods 
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      z = repmat(0,1,n);
      normal = repmat([0;0;1],1,n);
    end
    
    function writeWRL(obj,fptr)
      m=20; n=20;
      color1 = [204 102 0]/256;  % csail orange
      color1 = hex2dec({'ee','cb','ad'})/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      color2 = hex2dec({'cd','af','95'})/256;
      fprintf(fptr,'Transform {\n  translation %f %f 0\n  rotation 1 0 0 1.5708\n  children [\n',-m/2,n/2);
      fprintf(fptr,'Shape { geometry ElevationGrid {\n');
      %        fprintf(fptr,'  solid "false"\n');
      fprintf(fptr,'  xDimension %d\n',m);
      fprintf(fptr,'  zDimension %d\n',n);
      fprintf(fptr,'  height [');
      fprintf(fptr,' %d', zeros(m,n));
      fprintf(fptr,' ]\n');
      fprintf(fptr,'  colorPerVertex TRUE\n');
      fprintf(fptr,'   color Color { color [');
      for i=1:m
        for j=1:n
          if rem(i+j,2)==0
            fprintf(fptr,' %.1f %.1f %.1f,', color1);
          else
            fprintf(fptr,' %.1f %.1f %.1f,', color2);
          end
        end
      end
      fprintf(fptr,'] }\n');
      fprintf(fptr,'}\n}\n]\n}\n\n');
    end

    function geom = constructRigidBodyGeometry(obj)
      box_width = 1000;
      box_depth = 10;
      geom = RigidBodyBox([box_width;box_width;box_depth]);
      geom.T(3,4) = -box_depth/2;
      geom.c = [0.5,0.5,0.5];
    end
  end
end
