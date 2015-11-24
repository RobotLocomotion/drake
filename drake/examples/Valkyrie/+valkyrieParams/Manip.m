classdef Manip < valkyrieParams.Standing
  methods 
    function obj = Manip(r)
      obj = obj@valkyrieParams.Standing(r);
      l_hand = r.findLinkId('leftPalm');
      r_hand = r.findLinkId('rightPalm');
      obj.body_motion(l_hand).weight = 0.001;
      obj.body_motion(r_hand).weight = 0.001;
    end
  end
end
