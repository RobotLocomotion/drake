classdef AtlasCommandToEffortBlock < DrakeSystem
  methods
    function obj = AtlasCommandToEffortBlock(r)
      typecheck(r, 'Atlas');

      input_frame = drcFrames.AtlasState(r);
      output_frame = atlasFrames.AtlasInput(r);

      obj = obj@DrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = obj.setInputFrame(input_frame);
      obj = obj.setOutputFrame(output_frame);


    end

    function y = mimoOutput(obj, ~, ~, x)
      nu = length(obj.getOutputFrame.coordinates);
      y = x((end-nu+1:end));
    end
  end
end
