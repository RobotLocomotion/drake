classdef BodyMotionData
  properties
    ts;
    coefs;
    body_id;
    toe_off_allowed;
    in_floating_base_nullspace;
    control_pose_when_in_contact;
  end

  methods
    function obj = BodyMotionData(body_id, ts)
      obj.ts = ts;
      obj.body_id = body_id;
      obj.toe_off_allowed = false(1, numel(ts));
      obj.in_floating_base_nullspace = false(1, numel(ts));
      obj.control_pose_when_in_contact = false(1, numel(ts));
      obj.coefs = zeros(6, numel(ts), 4);
    end

    function t_ind = findTInd(obj, t)
      if t < obj.ts(1)
        t_ind = 1;
      elseif t >= obj.ts(end)
        t_ind = length(obj.ts) - 1;
      else
        t_ind = find(obj.ts <= t, 1, 'last');
      end
    end

    function [x, xd, xdd] = eval(obj, t)
      t_ind = obj.findTInd(t);
      [x, xd, xdd] = obj.evalAtInd(t_ind, t);
    end

    function [x, xd, xdd] = evalAtInd(obj, t_ind, t)
      t_rel = t - obj.ts(t_ind);
      x = obj.coefs(:,t_ind,1)*t_rel^3 + obj.coefs(:,t_ind,2)*t_rel^2 + obj.coefs(:,t_ind,3)*t_rel + obj.coefs(:,t_ind,4);
      xd = 3*obj.coefs(:,t_ind,1)*t_rel^2 + 2*obj.coefs(:,t_ind,2)*t_rel + obj.coefs(:,t_ind,3);
      xdd = 6*obj.coefs(:,t_ind,1)*t_rel + 2*obj.coefs(:,t_ind,2);
    end

    function body_motion_slice = sliceAtTime(obj, t)
      body_motion_slice = obj.slice(obj.findTInd(t));
    end

    function body_motion_slice = slice(obj, t_ind)
      slice_t_inds = min([t_ind, t_ind+1], [length(obj.ts), length(obj.ts)]);
      slice_t_inds = max(slice_t_inds, [1, 1]);
      body_motion_slice = BodyMotionData(obj.body_id, obj.ts(slice_t_inds));
      body_motion_slice.coefs = obj.coefs(:,t_ind,:);
      body_motion_slice.toe_off_allowed = obj.toe_off_allowed(t_ind);
      body_motion_slice.in_floating_base_nullspace = obj.in_floating_base_nullspace(t_ind);
      body_motion_slice.control_pose_when_in_contact = obj.control_pose_when_in_contact(t_ind);
    end
  end

  methods(Static)
    function obj = from_body_poses(body_id, ts, poses)
      obj = BodyMotionData(body_id, ts);
      pp = pchip(ts, poses);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end

    function obj = from_body_poses_and_velocities(body_id, ts, poses, dposes)
      obj = BodyMotionData(body_id, ts);

      pp = pchipDeriv(ts, poses, dposes);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end
  end
end

