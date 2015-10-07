classdef RigidBodyStepTerrain < RigidBodyHeightMapTerrain
%  This class provides an implementation of RigidBodyTerrain with a step
%  from z = 0.  Step is oriented in the x-axis.
%  Uses a smooth step function (3t^2 - 2t^3) that has a zero derivative at
%  x = 0 and x = 1
%  Currently returns normal with NON-UNIT value. This is because
%  phi(q) != distance, and the normal is used with forward kinematics to
%  calculate dphi/dq
%  WARNING! This class will not work, because it violates assumptions made
%  bu collision detection. Specifically, dnormal/dq != 0
  
  methods 
    function obj = RigidBodyStepTerrain(step_height,step_x,smoothing_width)
      x = -10:.01:10;
      y = [-5 5];
      [X,Y] = ndgrid(x,y);
      
      Z = zeros(length(x),length(y));
      t = (X - step_x + smoothing_width)/(2*smoothing_width);
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
      t = (xy(1,:) - obj.x - obj.smoothing)/(2*obj.smoothing);
      z(t >= 1) = obj.height;
      I_s = find(t > 0 & t < 1);
      z(I_s) = obj.height*(3*t(I_s).^2 - 2*t(I_s).^3);
      normal = repmat([0;0;1],1,n);
      normal(I_s,1) = 3*obj.height*(t(I_s) - t(I_s).^2)/obj.smoothing;
    end
  end
  
  properties
    height;
    x;
    smoothing;
    PTS;
  end
end
