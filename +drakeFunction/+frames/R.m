function frame = R(n)
  % frame = drakeFunction.frames.R(n) returns a singleton frame
  %   representing R^n
  %
  % @param n        -- Integer specifying the dimension of the frame.
  %
  % @retval frame   -- SingletonCoordinateFrame representing R^n
  name = sprintf('R%d',n);
  integervaluedcheck(n);
  frame = SingletonCoordinateFrame(name,n,'e');
end
