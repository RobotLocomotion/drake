function identityTest()
  import drakeFunction.*
  frame = CoordinateFrame('x',5);
  expr = Identity(frame);
  x = rand(5,1);
  [f,df] = expr.eval(x);
  valuecheck(f,x);
  valuecheck(df,eye(expr.input_frame.dim));
end
