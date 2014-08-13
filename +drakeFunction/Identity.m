classdef Identity < drakeFunction.Linear
  methods
    function obj = Identity(frame)
      obj = obj@drakeFunction.Linear(frame,frame,eye(frame.dim));
    end
  end
end
