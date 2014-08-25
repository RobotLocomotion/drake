function constantPowerTest()
  import drakeFunction.*
  frame = CoordinateFrame('x',3);

  % Test with scalar power
  fun = ConstantPower(frame,frame,2);
  x = rand(frame.dim,1);
  [f,df] = fun(x);
  valuecheck(f,x.^2);
  valuecheck(df,2*diag(x));

  % Test with vector power
  power = (1:3)';
  fun = ConstantPower(frame,frame,power);
  x = rand(frame.dim,1);
  [f,df] = fun(x);
  valuecheck(f,x.^power);
  valuecheck(df,diag(power.*x.^(power-1)));

  % Test with geval
  [f,df] = geval(@(x) eval(fun,x),x,struct('grad_method',{{'user','taylorvar','numerical'}},'tol',1e-6));
end
