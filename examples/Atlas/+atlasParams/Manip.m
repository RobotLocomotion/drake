classdef Manip < atlasParams.Standing
  methods 
    function obj = Manip(r)
      obj = obj@atlasParams.Standing(r);
      l_hand = r.findLinkId('l_hand');
      r_hand = r.findLinkId('r_hand');
      obj.body_motion(l_hand).weight = 0.001;
      obj.body_motion(r_hand).weight = 0.001;
      obj.body_motion(l_hand).damping_ratio = 0.9;
      obj.body_motion(r_hand).damping_ratio = 0.9;
      obj = obj.updateKd();
    end
  end
end
