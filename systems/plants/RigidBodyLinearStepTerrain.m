classdef RigidBodyLinearStepTerrain < RigidBodyHeightMapTerrain
%  This class provides an implementation of RigidBodyTerrain with a step
%  from z = 0.  Step is oriented in the x-axis.
%  Uses a linear step up/down.
%  Currently returns normal with NON-UNIT value. This is because
%  phi(q) != distance, and the normal is used with forward kinematics to
%  calculate dphi/dq
  
  methods 
    function obj = RigidBodyLinearStepTerrain(step_height,step_x,smoothing_width)
      x = -10:.01:10;
      y = [-5 5];
      [X,Y] = ndgrid(x,y);
      
      Z = zeros(length(x),length(y));
      t = (X - step_x + smoothing_width)/(2*smoothing_width);
      Z(t >= 1) = step_height;
      Z(t >0 & t < 1) = step_height*t(I_s);
      
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
      z(I_s) = obj.height*t(I_s);
      normal = repmat([0;0;1],1,n);
      normal(I_s,1) = obj.height/(2*obj.smoothing);
    end
  end
  
  properties
    height;
    x;
    smoothing;
    PTS;
  end
end
