classdef RigidBodyStepTerrain < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere, except on a series of boxes of center, height specified
%  on creation.
  
% Boxes specified as an Nx5 matrix with columns:
%  X center, Y center Width (x), Length (y), Height (z)
  methods 
    function obj = RigidBodyStepTerrain(boxes)
      if nargin < 1
        boxes = [1.0, 0.0, 1, 1, 0.05];
      end
      obj.centers_x = boxes(:, 1);
      obj.centers_y = boxes(:, 2);
      obj.boxs_x = boxes(:, 3);
      obj.boxs_y = boxes(:, 4);
      obj.boxs_z = boxes(:, 5);
      obj.num_boxes = size(boxes, 1);
      obj.geom = constructGeometry(obj);
    end
    
    function [z,normal] = getHeight(obj,xy)
      [~, n] = size(xy);
      x = repmat(xy(1, :), [obj.num_boxes, 1]);
      y = repmat(xy(2, :), [obj.num_boxes, 1]);
      
      x_above_boxes = x >= repmat(obj.centers_x - obj.boxs_x/2, [1, size(xy, 2)]);
      x_below_boxes = x <= repmat(obj.centers_x+ obj.boxs_x/2, [1, size(xy, 2)]);
      x_in_boxes = x_above_boxes & x_below_boxes;
      
      y_above_boxes = y >= repmat(obj.centers_y - obj.boxs_y/2, [1, size(xy, 2)]);
      y_below_boxes = y <= repmat(obj.centers_y+ obj.boxs_y/2, [1, size(xy, 2)]);
      y_in_boxes = y_above_boxes & y_below_boxes;
      
      in_boxes = x_in_boxes & y_in_boxes;
      
      % Give the height of the heighest box that each point falls in
      adjusted_heights = repmat(obj.boxs_z, [1, size(xy, 2)]) .* in_boxes;
      z = max(adjusted_heights, [], 1);
      
      % Normals always straight up
      normal = repmat([0;0;1],1,n);
    end

    function geom = getCollisionGeometry(obj)
      geom = obj.geom;
    end

    function geom = getVisualGeometry(obj)
      geom = obj.geom;
    end
    
    function geom = constructGeometry(obj)
      ground_width = 1000;
      ground_depth = 10;
      geom_ground = RigidBodyBox([ground_width;ground_width;ground_depth]);
      geom_ground.T(3,4) = -ground_depth/2;
      geom_ground.c = hex2dec({'ee','cb','ad'})'/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      geom_ground.name = 'terrain';
%      geom.c = hex2dec({'cd','af','95'})'/256;
      
      geom = cell(obj.num_boxes + 1, 1);
      geom{1} = geom_ground;
      for i=1:obj.num_boxes
        geom_box = RigidBodyBox([obj.boxs_x(i); obj.boxs_y(i); obj.boxs_z(i)]);
        geom_box.T(1:3, 4) = [obj.centers_x(i); obj.centers_y(i); obj.boxs_z(i)/2];
        geom_box.c = hex2dec({'ee','cb','ad'})'/256;
        geom_box.name = 'terrain';
      
        geom{i+1} = geom_box;
      end
    end
  end
  
  properties
    geom;
    centers_x;
    centers_y;
    boxs_x;
    boxs_y;
    boxs_z;
    num_boxes;
  end
end
