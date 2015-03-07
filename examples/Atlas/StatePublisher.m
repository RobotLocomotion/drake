classdef StatePublisher < DrakeSystem
  properties
    lc
    lcmcoder
  end

  methods
    function obj = StatePublisher(r)
      input_frame = r.getOutputFrame();
      output_frame = input_frame;
      obj = obj@DrakeSystem(0, 0, numel(input_frame.coordinates), numel(output_frame.coordinates), true, true);
      obj = obj.setInputFrame(input_frame);
      obj = obj.setOutputFrame(output_frame);
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmcoder = drcFrames.AtlasState(r).lcmcoder;
    end

    function y = output(obj, t, x, u)
      y = u;
      obj.lc.publish('EST_ROBOT_STATE', obj.lcmcoder.encode(t, u));
    end
  end
end
