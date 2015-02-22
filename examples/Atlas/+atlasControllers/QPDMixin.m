classdef QPDMixin
  properties
    qddot_des_min
    qddot_des_max
  end

  methods
    function obj = QPDMixin(r,qddot_range)
      if all(size(qddot_range) == [1,2])
        obj.qddot_des_min = qddot_range(1) * ones(r.getNumPositions(), 1);
        obj.qddot_des_max = qddot_range(2) * ones(r.getNumPositions(), 1);
      elseif all(size(qddot_range) == [r.getNumPositions(), 2])
        obj.qddot_des_min = qddot_range(:,1);
        obj.qddot_des_max = qddot_range(:,2);
      else
        % will always fail; that's expected (we just want to throw a useful error)
        sizecheck(qddot_range, [1,2]);
      end
    end

    function qddot_des = getQddot_des(obj, q, qd, q_des, params)
      err_q = [q_des(1:3) - q(1:3); angleDiff(q(4:end), q_des(4:end))];
      qddot_des = params.Kp .* err_q - params.Kd .* qd;
      qddot_des = max(obj.qddot_des_min,...
                      min(obj.qddot_des_max, qddot_des));
    end
  end
end
