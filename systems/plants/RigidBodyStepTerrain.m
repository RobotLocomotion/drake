classdef RigidBodyStepTerrain < RigidBodyHeightMapTerrain
%  This class provides an implementation of RigidBodyTerrain with a step
%  from z = 0.  Step is oriented in the x-axis.
%  Uses a smooth step function (3t^2 - 2t^3) that has a zero derivative at
%  x = 0 and x = 1
%  Currently returns normal = z, but this could be changed if desired.
  
  methods 
    function obj = RigidBodyStepTerrain(step_height,step_x,smoothing_width)
      x = -10:.01:10;
      y = [-5 5];
      [X,Y] = ndgrid(x,y);
      
      Z = zeros(length(x),length(y));
      t = (X - step_x)/smoothing_width;
      Z(t >= 1) = step_height;
      Z(t >0 & t < 1) = step_height*(3*t(I_s).^2 - 2*t(I_s).^3);
      
      obj@RigidBodyHeightMapTerrain(x,y,Z',eye(4));
        
      obj.height = step_height;
      obj.x = step_x;
      obj.smoothing = smoothing_width;
      obj.geom = constructRigidBodyGeometry(obj);
    end    
    
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      
      z = zeros(1,n);
      t = (xy(1,:) - obj.x)/obj.smoothing;
      z(t >= 1) = obj.height;
      z(t > 0 & t < 1) = obj.height*(3*t(I_s).^2 - 2*t(I_s).^3);
      normal = repmat([0;0;1],1,n);
    end
  end
  
  properties
    height;
    x;
    smoothing;
    PTS;
  end
end
