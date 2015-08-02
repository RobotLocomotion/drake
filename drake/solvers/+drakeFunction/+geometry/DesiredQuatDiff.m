classdef DesiredQuatDiff<drakeFunction.DrakeFunction
  % Given a quaternion quat, and the desired quaternion quat_des, compute the geodesic distance
  % between those two quaternions as 2(quat'*quat_des)^2-1
  properties(SetAccess = protected)
    quat_des % The desired quaternion. A 4x1 vector
  end
  
  methods
    function obj = DesiredQuatDiff(quat_des)
      if(~isnumeric(quat_des))
        error('Drake:DesiredQuatDiff:InvalidInputs','quat_des should be numeric');
      end
      if(any(isinf(quat_des)|isnan(quat_des)))
        error('Drake:DesiredQuatDiff:InvalidInputs','quat_des should not contain inf or nan');
      end
      sizecheck(quat_des,[4,1]);
      norm_quat_des = norm(quat_des);
      valuecheck(norm_quat_des,1,1e-4);
      quat_des = quat_des/norm_quat_des;
      dim_input = 4;
      dim_output = 1;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
      obj.quat_des = quat_des;
    end
    
    function [distance,ddistance] = eval(obj,quat)
      % computes the geodesic distance between quaternion quat and desired quaternion obj.quat_des
      % as distance = 2(quat'*obj.quat_des)^2-1
      inner_product = quat'*obj.quat_des;
      distance = 2*inner_product^2-1;
      if(nargout>1)
        ddistance = 4*inner_product*obj.quat_des';
      end
    end
  end
end
