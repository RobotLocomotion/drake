classdef Walking < atlasParams.Base
  methods
    function obj = Walking(r)
      typecheck(r, 'Atlas');
      obj = obj@atlasParams.Base(r);
      obj.contact_threshold = 0.001;
      obj.whole_body.w_qdd = zeros(r.getNumVelocities(), 1);
      if (r.getNumVelocities() ~= r.getNumPositions())
        error('this code calls findPositionIndices, which is no longer equivalent to findVelocityIndices');
      end
      obj.whole_body.w_qdd(findPositionIndices(r, 'leg')) = 1e-6;
      obj.whole_body.w_qdd(findPositionIndices(r, 'arm')) = .0001;
      obj.whole_body.w_qdd(findPositionIndices(r, 'back')) = .0001;
      obj.whole_body.w_qdd(r.findPositionIndices('back_bkx')) = 0.1;
      obj.whole_body.damping_ratio = 0.5;
      obj.body_motion(r.findLinkId('pelvis')).Kp = [0; 0; 150; 200; 200; 200];
      obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.6;
      obj.body_motion(r.foot_body_id.right).Kp = [100; 100; 100; 150; 150; 150];
      obj.body_motion(r.foot_body_id.right).damping_ratio = 0.5;
      obj.body_motion(r.foot_body_id.left).Kp = [100; 100; 100; 150; 150; 150];
      obj.body_motion(r.foot_body_id.left).damping_ratio = 0.5;
      [obj.body_motion.weight] = deal(0.001);
      obj.body_motion(r.findLinkId('pelvis')).weight = 0.075;
      obj.Kp_accel = 0;
      obj = obj.updateKd();
    end
  end
end



