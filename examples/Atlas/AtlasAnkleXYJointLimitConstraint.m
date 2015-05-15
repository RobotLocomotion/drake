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
      ankle_limits_A = [0.5044   -0.8635;    0.1059   -0.9944;    1.0000    0.0000;   -0.1083   -0.9941;   -0.5044   -0.8635;    0.4510    0.8925;   -1.0000   -0.0000;   -0.4555    0.8902];
      ankle_limits_b = [1.0253;    1.0137;    0.6411;    1.0143;    1.0253;    0.6163;    0.6411;    0.6183];
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