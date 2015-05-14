classdef AtlasAnkleXYJointLimitConstraint < SingleTimeLinearPostureConstraint
  properties(SetAccess = protected)
    % ankle_limits_A*[akx;aky]<= ankle_limits_b;
    ankle_limits_A
    ankle_limits_b
  end
  
  methods
    function obj = AtlasAnkleXYJointLimitConstraint(robot)
      l_leg_akx = robot.findPositionIndices('l_leg_akx');
      l_leg_aky = robot.findPositionIndices('l_leg_aky');
      r_leg_akx = robot.findPositionIndices('r_leg_akx');
      r_leg_aky = robot.findPositionIndices('r_leg_aky');
      ankle_limits_A = [0.0694   -0.9976;   -0.0694   -0.9976;    0.5183   -0.8552;   -0.5183   -0.8552;    1.0000   -0.0000;   -1.0000   -0.0000;    0.7107    0.7035;    0.5090    0.8608;   -0.7107    0.7035;   -0.5090    0.8608];
      ankle_limits_b = [0.9976;    0.9976;    0.9871;    0.9871;    0.4360;    0.4360;    0.5913;    0.6198;    0.5913;    0.6198];
      num_lin_ineq = length(ankle_limits_b);
      iAfun = [(1:num_lin_ineq)';(1:num_lin_ineq)';num_lin_ineq+(1:num_lin_ineq)';num_lin_ineq+(1:num_lin_ineq)'];
      jAvar = reshape(bsxfun(@times,ones(num_lin_ineq,1),[l_leg_akx l_leg_aky r_leg_akx r_leg_aky]),[],1);
      Aval = reshape([ankle_limits_A ankle_limits_A],[],1);
      obj = obj@SingleTimeLinearPostureConstraint(robot,iAfun,jAvar,Aval,-inf(2*num_lin_ineq,1),[ankle_limits_b;ankle_limits_b]);
      obj.ankle_limits_A = ankle_limits_A;
      obj.ankle_limits_b = ankle_limits_b;
    end
  end
end