classdef Walking < valkyrieParams.Base
  methods
    function obj = Walking(r)
      typecheck(r, 'Valkyrie');
      obj = obj@valkyrieParams.Base(r);
      obj.contact_threshold = 0.001;
      obj.whole_body.w_qdd = zeros(r.getNumVelocities(), 1);
      if (r.getNumVelocities() ~= r.getNumPositions())
        error('this code calls findPositionIndices, which is no longer equivalent to findVelocityIndices');
      end

      obj.whole_body.w_qdd(findPositionIndices(r, 'Hip')) = 1e-6;
      obj.whole_body.w_qdd(findPositionIndices(r, 'Knee')) = 1e-6;
      obj.whole_body.w_qdd(findPositionIndices(r, 'Ankle')) = 1e-6;

      obj.whole_body.w_qdd(findPositionIndices(r, 'Shoulder')) = .0001;
      obj.whole_body.w_qdd(findPositionIndices(r, 'Elbow')) = .0001;
      obj.whole_body.w_qdd(findPositionIndices(r, 'Forearm')) = .0001;
      obj.whole_body.w_qdd(findPositionIndices(r, 'Wrist')) = .0001;

      obj.whole_body.w_qdd(findPositionIndices(r, 'torso')) = .0001;

      obj.whole_body.w_qdd(r.findPositionIndices('torsoRoll')) = 0.1;

      %This gain was added specifically for Valkyrie.
      % There must be something else going on as it shouldn't be needed.
      obj.whole_body.w_qdd(findPositionIndices(r, 'lowerNeckPitch')) = .0001;
      
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



