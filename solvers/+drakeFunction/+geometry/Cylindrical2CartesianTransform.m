classdef Cylindrical2CartesianTransform < drakeFunction.DrakeFunction
  % Given a cylindrical coordinate frame, transform its coordinates to a
  % Cartesian coordinate. Refer to doc_cartesian2cylindrical.pdf for the
  % transformation
  properties(SetAccess = protected)
    T_cylinder % This is a homogeneous transformation matrix, T_cylinder transforms a cylinder with [0;0;1] being its axis, and nd the angle theta measured with respect to the x-axis
  end
  
  methods
    function obj = Cylindrical2CartesianTransform(T_cylinder)
      input_frame = drakeFunction.frames.realCoordinateSpace(6);
      output_frame = drakeFunction.frames.realCoordinateSpace(6);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      if(~isHT(T_cylinder))
        error('T_cylinder should be a homogenous transformation matrix');
      end
      obj.T_cylinder = T_cylinder;
    end
    
    function [x_cartesian,J] = eval(obj,x_cylinder)
      % Refer to doc_cartesian2cylindrical.pdf for the transformation
      % @retval x_cartesian = [x;y;z;roll;pitch;yaw];
      % @param x_cylinder =
      % [radius;theta;height;roll_tangent,pitch_tangent,yaw_tangent]
      radius = x_cylinder(1);
      theta = x_cylinder(2);
      height = x_cylinder(3);
      x_pos_cartesian = obj.T_cylinder(1:3,1:3)*[radius*cos(theta);radius*sin(theta);height]+obj.T_cylinder(1:3,4);
      dx_pos_cartesian = obj.T_cylinder(1:3,1:3)*[[cos(theta) radius*-sin(theta) 0;sin(theta) radius*cos(theta) 0;0 0 1] zeros(3,3)];
      [R_tangent,dR_tangent] = rpy2rotmat(x_cylinder(4:6));
      dR_tangent = [zeros(9,3) dR_tangent];
      [R_tangent2cylinder,dR_tangent2cylinder] = rotz(pi/2-theta);
      dR_tangent2cylinder = [zeros(9,1) -dR_tangent2cylinder(:) zeros(9,4)];
      R_cylinder = R_tangent2cylinder*R_tangent;
      dR_cylinder = matGradMultMat(R_tangent2cylinder,R_tangent,dR_tangent2cylinder,dR_tangent);
      [x_rpy_cartesian,dx_rpy_cartesian] = rotmat2rpy(obj.T_cylinder(1:3,1:3)*R_cylinder,reshape(obj.T_cylinder(1:3,1:3)*reshape(dR_cylinder,3,[]),9,[]));
      x_cartesian = [x_pos_cartesian;x_rpy_cartesian];
      J = [dx_pos_cartesian;dx_rpy_cartesian];
    end
  end
end