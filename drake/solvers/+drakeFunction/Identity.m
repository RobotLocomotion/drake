classdef Identity < drakeFunction.Linear
  % DrakeFunction representing the identity transform in a given frame
  methods
    function obj = Identity(dim_input)
      % obj = Identity(frame) returns an identity transform for the
      % CoordinateFrame, 'frame'.
      %
      % @param frame  -- CoordinateFrame object
      obj = obj@drakeFunction.Linear(eye(dim_input));
    end
  end
end
