classdef CommandReceiver < DrakeSystem
  properties
    lcm_frame
  end

  methods
    function obj = CommandReceiver(r)
      input_frame = r.getInputFrame();
      output_frame = input_frame;
      obj = obj@DrakeSystem(0,0,numel(input_frame.coordinates),numel(output_frame.coordinates),true,true);
      obj = obj.setInputFrame(input_frame);
      obj = obj.setOutputFrame(output_frame);
      obj.lcm_frame = drcFrames.AtlasInput(r);
      obj.lcm_frame.subscribe('ATLAS_COMMAND');
    end

    function y = output(obj, t, x, u)
      [u_lcm, t_lcm] = obj.lcm_frame.getCurrentValue();
      if isempty(u_lcm)
        y = u;
      else
        y = u_lcm(end-27:end);
      end
    end
  end
end

