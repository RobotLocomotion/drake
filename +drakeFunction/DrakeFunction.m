classdef DrakeFunction
  % drakeFunction.DrakeFunction   Abstract parent class for all DrakeFunction classes
  % A DrakeFunction represents a vector-valued function that maps a
  % point in one CoordinateFrame to a point in another CoordinateFrame
  
  properties (SetAccess = immutable)
    input_frame   % CoordinateFrame representing the domain
    output_frame  % CoordinateFrame representing the range
  end

  methods (Abstract)
    % All child classes must implement an 'eval' method that evaluates
    % the function and its derivative
    [f,df] = eval(obj,x);
  end

  methods
    function obj = DrakeFunction(input_frame,output_frame)
      % obj = drakeFunction.DrakeFunction(input_frame,output_frame)
      %   returns a DrakeFunction object representing a map from
      %   input_frame to output_frame
      %
      % @param input_frame    -- CoordinateFrame representing the domain
      % @param output_frame   -- CoordinateFrame representing the range
      %
      % @retval obj           -- drakeFunction.DrakeFunction object

      typecheck(input_frame,'CoordinateFrame');
      typecheck(output_frame,'CoordinateFrame');
      obj.input_frame = input_frame;
      obj.output_frame = output_frame;
    end

    function input_frame = getInputFrame(obj)
      input_frame = obj.input_frame;
    end

    function output_frame = getOutputFrame(obj)
      output_frame = obj.output_frame;
    end
  end
end
