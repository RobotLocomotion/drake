classdef Sum < drakeFunction.Root
  methods
    function obj = Sum(frame,N)
      % obj = Sum(input_frame) constructs a drakeFunction.Sum object
      input_frame = MultiCoordinateFrame(repmat({frame},1,N));
      output_frame = frame;
      obj = obj@drakeFunction.Root(input_frame,output_frame);
    end
    function [f,df] = eval(obj,x)
      x_cell = splitCoordinates(obj.input_frame,x);
      f = sum(horzcat(x_cell{:}),2);
      df = repmat(eye(numel(f)),1,numel(x_cell));
    end
  end
end
