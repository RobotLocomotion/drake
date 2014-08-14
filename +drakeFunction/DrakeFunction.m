classdef DrakeFunction
  properties (SetAccess = immutable)
    input_frame;
    output_frame;
  end
  methods (Abstract)
    [f,df] = eval(obj,x);
  end
  methods
    function obj = DrakeFunction(input_frame,output_frame)
      obj.input_frame = input_frame;
      obj.output_frame = output_frame;
    end

    function input_frame = getInputFrame(obj)
      input_frame = obj.input_frame;
    end

    function output_frame = getOutputFrame(obj)
      output_frame = obj.output_frame;
    end

    function obj = setInputFrame(obj,input_frame)
      obj.input_frame = input_frame;
    end

    function obj = setOutputFrame(obj,output_frame)
      obj.output_frame = output_frame;
    end

  end
end
