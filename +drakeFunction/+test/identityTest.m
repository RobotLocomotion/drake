function identityTest()
  import drakeFunction.*
  frame = CoordinateFrame('x',5);
  fcn = Identity(frame);
  x = rand(5,1);
  [f,df] = fcn(x);
  valuecheck(f,x);
  valuecheck(df,eye(fcn.input_frame.dim));
end
