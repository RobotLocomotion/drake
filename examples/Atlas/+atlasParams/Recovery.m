classdef Recovery < atlasParams.Walking
  methods
    function obj = Recovery(r)
      obj = obj@atlasParams.Walking(r);
      obj.body_motion(r.foot_body_id.right).Kp = 10 * [100; 100; 100; 150; 150; 150];
      obj.body_motion(r.foot_body_id.left).Kp = 10 * [100; 100; 100; 150; 150; 150];
      obj.body_motion(r.foot_body_id.right).weight = 1e-3;
      obj.body_motion(r.foot_body_id.left).weight = 1e-3;
      obj.body_motion(r.findLinkId('pelvis')).weight = 1e-2;

      w_qdd = zeros(r.getNumVelocities(), 1);
      w_qdd(r.findPositionIndices('arm')) = 1e-4;
      w_qdd(r.findPositionIndices('back')) = 1e-4;
      obj.whole_body.w_qdd = w_qdd;
      obj = obj.updateKd();
    end
  end
end



