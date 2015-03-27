classdef Manip < atlasParams.Standing
  methods 
    function obj = Manip(r)
      obj = obj@atlasParams.Standing(r);
      obj.body_motion(r.findLinkId('r_hand')).weight = 0.001;
      obj.body_motion(r.findLinkId('l_hand')).weight = 0.001;
    end
  end
end
